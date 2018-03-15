import numpy as np
import matplotlib.pyplot as plt

from fvks.scenario.trajectory import StateTupleFactory
from fvks.scenario.trajectory import Trajectory
from fvks.scenario.trajectory import collision_object_from_trajectory
from fvks.visualization.draw_dispatch import draw_object
import pyfvks


class StatePropagator:
    """
    Propagates a system according to its dynamics given an initial state and
    an input for the time range state.time_begin*dt to duration_trajectory*dt.
    """

    def __init__(self, vehicle_model, duration_trajectory, dt):
        """
        :param vehicle_model:         fvks.planning.vehicle_models.Vehicle
        :param duration_trajectory:   duration of one propagated trajectory
        :param dt:                    time discretization of scenario
        """
        self.vehicle_model = vehicle_model
        self.duration_trajectory = duration_trajectory
        self.dt = dt

    def initialize(self, state):
        """
        :param state: core initial state from CommonRoad XML-file
        :return: full initial state of vehicle model
        """
        return self.vehicle_model.initialize(state)

    def steer(self, state_begin, state_end, distance_func):
        """
        Computes the trajectory that takes the system from state_begin near state_end.

        :param state_begin:     starting state of trajectory
        :param state_end:       final state which should be reached
        :param distance_func:   distance function between two states given by StateSampler
        :return:                fvks.scenario.trajectory.Trajectory steering vehicle from
                                state_begin near state_end
        """
        diff = np.infty

        # sample 20 different inputs and take the input which steers the system nearest to
        # state_end
        n = 20
        while n:
            # randomly sample inputs
            u = self._sample_inputs()
            inputs = u*np.ones((self.duration_trajectory, 2))
            # propagate system dynamics
            trajectory_, collision_object_, constraints_fulfilled = self.vehicle_model.forward_simulation(
                state_begin, state_begin.time, inputs, self.dt)

            # if constraints on vehicle dynamic are not fulfilled compute new trajectory
            if not constraints_fulfilled:
                continue

            # if new the distance between the final state of the new trajectory and state_end is less than
            # the distance of the current best trajectory to state_end, store new trajectory.
            diff_new = distance_func(trajectory_.state_list[-1], state_end)
            if diff > diff_new:
                trajectory = trajectory_
                collision_object = collision_object_
                diff = diff_new
            # decrease counter
            n -= 1
        return trajectory, collision_object

    @classmethod
    def _sample_inputs(cls):
        """
        Randomly samples the inputs steering velocity and acceleration.
        """
        a = - 2 + 2 * 2 * np.random.rand()
        v_steering = - 0.2 + 2 * 0.2 * np.random.rand()
        return [v_steering, a]


class StateSamplerPosition:
    """
    StateSamplerPosition draws a sample in the position space.
    """
    state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position)

    def __init__(self, x_range, y_range, ):
        """
        :param x_range: valid sampling range in x-direction
        :param y_range: valid sampling range in y-direction 
        """
        self.x_range = x_range
        self.y_range = y_range

    def sample(self):
        """
        Samples a new state uniformly within x_range and y_range.
        :return: state created by fvks.scenario.trajectory.StateTupleFactory
        """
        sample_x = self.x_range[0] + (self.x_range[1] - self.x_range[0]) * np.random.rand()
        sample_y = self.y_range[0] + (self.y_range[1] - self.y_range[0]) * np.random.rand()
        sampled_state = StateSamplerPosition.state_tuple(np.array([sample_x, sample_y]))
        return sampled_state

    @classmethod
    def create_from_goal_state(cls, goal_state):
        """
        Creates a sample from the goal region. Currently only goal regions which position
        is described by a pyfvks.collision.RectOBB with orientation 0.0 are supported.
        :param goal_state: goal state of a planning problem
        :return: sample in goal region
        """
        if type(goal_state.position) == pyfvks.collision.RectOBB:
            # sample in goal region
            x_range = [goal_state.position.center()[0] - goal_state.position.r_x(),
                       goal_state.position.center()[0] + goal_state.position.r_x()]
            y_range = [goal_state.position.center()[1] - goal_state.position.r_y(),
                       goal_state.position.center()[1] + goal_state.position.r_y()]
            sample_x = x_range[0] + (x_range[1] - x_range[0]) * np.random.rand()
            sample_y = y_range[0] + (y_range[1] - y_range[0]) * np.random.rand()
            sampled_state = StateSamplerPosition.state_tuple(np.array([sample_x, sample_y]))
        else:
            raise Exception()
        return sampled_state

    @classmethod
    def distance(cls, start, goal):
        """
        Computes the Euclidean distance between two states.
        :param start: first state
        :param goal:  second state
        :return: distance
        """
        return np.linalg.norm(start.position - goal.position)


class RRT:
    """
    Rapidly-exploring random tree. Searches a trajectory from a given starting state to a goal
    region.
    """

    class Node:
        """
        Represents a node in the RRT planner.
        """

        def __init__(self, state, parent, trajectory, child=None):
            """
            :param state:       state created with fvks.scenario.trajectory.StateTupleFactory
            :param parent:      parent node
            :param trajectory:  fvks.scenario.trajectory.Trajectory leading from parent to
                                state
            :param child:       child nodes
            """
            self.state = state
            assert (type(parent) == RRT.Node or parent is None)
            self.parent = parent
            self.trajectory = trajectory
            self.children = list()
            if child:
                self.children.append(child)

    def __init__(self, state_sampler, state_propagator, planning_problem, dt):
        """
        :param state_sampler:       samples a new state
        :param state_propagator:    propagates system dynamic for specific time range
        :param planning_problem:    planning problem from CommonRoad XML-file
        :param dt:                  time discretization of scenario
        """
        self.state_sampler = state_sampler
        self.state_propagator = state_propagator
        self.root_node = self.Node(self.state_propagator.initialize(planning_problem.initial_state),
                                   None, None)
        self.node_list = [self.root_node]
        self.planning_problem = planning_problem
        self.dt = dt

        self.samples = list()

    def find_closest_node(self, state):
        """
        Finds the closest node within the newly created nodes of the previous time step.
        :param state: state created with fvks.scenario.trajectory.StateTupleFactory
        :return:      node with minimal distance to state
        """
        # list of nodes from previous time step
        node_min = self.root_node
        dist_min = self.state_sampler.distance(node_min.state, state)
        # find the node in last_nodes_list with minimal distance to state
        for n in self.node_list:
            dist = self.state_sampler.distance(n.state, state)
            if dist < dist_min:
                dist_min = dist
                node_min = n
        return node_min

    def add_node(self, node):
        """
        :param node: node which should be added to the tree
        """
        self.node_list.append(node)
        node.parent.children.append(node)

    def plan(self, n_max_sampling, collision_checker, scenario):
        """
        :param n_max_sampling:      maximum number of samples drawn
        :param collision_checker:   collision checker for scenario
        :param scenario:            CommonRoad scenario
        :return:
        """
        for i in range(n_max_sampling):
            # every ten steps select a goal state randomly an try to connect it with the tree
            if i % 10 != 0:
                sampled_state = self.state_sampler.sample()
            else:
                idx = np.random.randint(0, len(self.planning_problem.goal.state_list))
                sampled_state = self.state_sampler.create_from_goal_state(
                    self.planning_problem.goal.state_list[idx])

            # stores samples for drawing
            self.samples.append(sampled_state)
            # find the nearest node in the tree for the sampled state
            closest_node = self.find_closest_node(sampled_state)

            # find trajectory connecting the sampled state with the closest node
            # in tree and add time information of sampled_state
            trajectory, collision_object = self.state_propagator.steer(
                closest_node.state, sampled_state, self.state_sampler.distance)

            if not collision_checker.collide(collision_object):
                new_node = RRT.Node(trajectory.state_list[-1], closest_node, trajectory)
                self.add_node(new_node)

                if self.planning_problem.goal_reached(trajectory, scenario)[0]:
                    print("Reached goal position")
                    self.planning_problem.add_solution(self._get_trajectory(new_node),
                                                       scenario.dt)
                    return


    @classmethod
    def _get_trajectory(cls, goal_node):
        """
        :param goal_node: current node which reaches the goal
        :return: fvks.scenario.Trajectory which leads from the initial state to the goal state
        """
        current_node = goal_node
        state_list = list()

        # go backwards from current node to all its parents and concatenate trajectory
        while True:
            if current_node.parent is not None:
                state_list += current_node.trajectory.state_list[::-1][:-1]
                current_node = current_node.parent
            else:
                state_list.append(current_node.state)
                return Trajectory(state_list[-1].time, state_list[::-1],
                                  goal_node.trajectory.state_tuple)

    def draw(self):
        """
        Method to draw the tree which is built during the motion planning with RRT.
        First, all sampled states are plotted and then all trajectories connecting
        the nodes.
        """
        for s in self.samples:
            plt.plot(s.position[0], s.position[1], '*g')

        for n in self.node_list:
            if n.trajectory:
                draw_object(collision_object_from_trajectory(n.trajectory))