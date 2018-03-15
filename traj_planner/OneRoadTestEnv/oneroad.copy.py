import os
import matplotlib.pyplot as plt
from fvks.visualization.draw_dispatch import draw_object
from fvks.scenario.commonroad.file_reader import CommonRoadFileReader
import pyfvks
import numpy as np
import math
from fvks.planning.planning import PlanningProblem
from fvks.scenario.trajectory import StateTupleFactory
from fvks.scenario.commonroad.util import UncertainIntervalScalar
from fvks.planning.vehicle_models import VehicleModelFactory

from rrt import StatePropagator, StateSamplerPosition, RRT


class Obstacles:
    """
    Obstacles draws all obstacles in the position space and also contain distance function.
    """
    state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position)

    def __init__(self, vertical_length, horizontal_length):

        self.vertical_length = vertical_length
        self.horizontal_length = horizontal_length

    def create_obstacle(self, scenario_lanlet_length, scenario):

        start_point = [scenario_lanlet_length[1] / 2, 0]
        x_length = scenario_lanlet_length[1] / 3
        y_length = scenario_lanlet_length[0] / 8
        obstacleLeft = pyfvks.collision.RectAABB(x_length, y_length, start_point[0], start_point[1])

        start_point = [scenario_lanlet_length[1] / 2, scenario_lanlet_length[0]]
        x_length = scenario_lanlet_length[1] / 2
        obstacleRight = pyfvks.collision.RectAABB(x_length, y_length, start_point[0], start_point[1])


        scenario.add_objects(obstacleLeft)
        scenario.add_objects(obstacleRight)

        return scenario

    def distance(self, start, goal):
        """
        Computes the Euclidean distance between two states.
        :param start: first state
        :param goal:  second state
        :return: distance
        """
        return np.linalg.norm(start.position - goal.position)

    def pseudo_distance(self, point1, point2, point3, point4, veh_position):
        """
        Computes the Pseudo distance between two states.
        :param point1: first point of polygon: it is a point so it is array of x and y
        :param point2: second point of polygon: it is a point so it is array of x and y
        :param tangents_vector1: first predefined tangent: it is a vector so it is array of x and y
        :param tangents_vector2: second predefined tangent: it is a vector so it is array of x and y
        :param veh_position: vehicle position: it is a vector so it is array of x and y
        :return: distance
        """

        tangents_vector1, tangents_vector2 = self.pseudo_distance_tangnet(point1,point2,point3,point4)

        a = (tangents_vector2[0] - tangents_vector1[0]) * (point1[0] - point2[0]) + \
            (tangents_vector2[1] - tangents_vector1[1]) * (point1[1] - point2[1])

        b = (veh_position[0] - point1[0]) * (tangents_vector2[0] - tangents_vector1[0]) + tangents_vector1[0] * (point1[0] - point2[0]) + \
            (veh_position[1] - point1[1]) * (tangents_vector2[1] - tangents_vector1[1]) + tangents_vector1[1] * (point1[1] - point2[1])

        c = (veh_position[0] - point1[0]) * tangents_vector1[0] + (veh_position[1] - point1[1]) * tangents_vector1[1]

        # it is quadratic equation so it will create two lambda, which one is valid!!!
        # lambda1 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        # lambda2 = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)

        # should we calculate X(lambda) and T(lambda)???
        # just special case of lambda as in paper
        lambdaOriginal = (tangents_vector1[1]*veh_position[1]+veh_position[0])/\
                         ((tangents_vector1[1]-tangents_vector2[1])*veh_position[1]+point3[0])

        return [veh_position[0]-lambdaOriginal*point3[0]+(1-lambdaOriginal)*point2[0],
         veh_position[1] - lambdaOriginal * point3[1] + (1 - lambdaOriginal) * point2[1]]


    def pseudo_distance_tangnet(self, point0, point1, point2, point3):
        tangent1 =np.array([(point2[0]-point0[0])/2*math.sqrt( ((point2[0]-point0[0])**2)+((point2[1]-point0[1])**2) ),
             (point2[1]-point0[1])/2*math.sqrt( ((point2[0]-point0[0])**2)+((point2[1]-point0[1])**2) )])
        tangent2 = np.array([(point3[0]-point1[0])/2*math.sqrt( ((point3[0]-point1[0])**2)+((point3[1]-point1[1])**2) ),
             (point3[1]-point1[1])/2*math.sqrt( ((point3[0]-point1[0])**2)+((point3[1]-point1[1])**2) )])
        tangent1 = tangent1 / tangent1[0]
        tangent2 = tangent2 / tangent2[0]
        return tangent1,tangent2

filename = os.getcwd() +"/OneRoad.xml"
print(filename)
scenario, planning_task = CommonRoadFileReader(filename).open()
scenario_lanlet_length=[6, 100]

#obstacleClass = Obstacles(scenario_lanlet_length[0], scenario_lanlet_length[1])

#scenario = obstacleClass.create_obstacle(scenario_lanlet_length, scenario)

#print(obstacleClass.pseudo_distance([0,1],[0,0],[1,0],[1,1],[0.5,1]))

# create planning problem
# Create state tuple for the initial state according to CommonRoad
initial_state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position,
                                                           StateTupleFactory.orientation,
                                                           StateTupleFactory.velocity,
                                                           StateTupleFactory.yawRate,
                                                           StateTupleFactory.slipAngle,
                                                           StateTupleFactory.time)

# ------------- Create the initial state of the new planning problem using the specified values  -------------
initial_state = initial_state_tuple(np.array((1.5, 1.5)),
                                    0,
                                    0,
                                    0.0,
                                    0.0,
                                    1.0/scenario.dt)
# ------------- Create the state tuple for the goal states  -------------
goal_state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position,
                                                        StateTupleFactory.orientation,
                                                        StateTupleFactory.time)
goal_state = list()
                                    ## width/2, height/2, orientation, x-position , y-position
goal_state.append(goal_state_tuple(pyfvks.collision.RectOBB(2, 1, 0.0, 95.0, 1.5),
                                UncertainIntervalScalar(-2.0, -1.0),
                                UncertainIntervalScalar(0.0/scenario.dt, 100.0/scenario.dt)))

new_planning_problem = PlanningProblem(planning_problem_id=1,
                                       initial_state=initial_state,
                                       goal_states=goal_state)
planning_task.add_planning_problem(new_planning_problem)

# maximum number of samples
n_max_sampling = 200
# create collision checker from scenario
collision_checker = scenario.create_collision_checker()

# define state space for sampling
state_space = StateSamplerPosition([0, 100], [0, 5])
# select vehicle model and the parameters
vehicle_model = VehicleModelFactory.create('KS', 1)
# state propagator steers vehicle from a start state
# to a goal state. Each propagated trajectory has
# duration of duration_trajectory*scenario.dt
duration_trajectory = 8
state_propagator = StatePropagator(vehicle_model, duration_trajectory, scenario.dt)

plt.figure(figsize=(25, 25))

# solve each planning problem sequentially with RRT
for i, p in enumerate(planning_task.planning_problems):
    print('Solving planning problem ', p.planning_problem_id)

    # initialize RRT and plan trajectory
    rrt = RRT(state_space, state_propagator, p, scenario.dt)
    rrt.plan(n_max_sampling, collision_checker, scenario)

    # plot tree of RRT
    plt.subplot(1, len(planning_task.planning_problems), i + 1)
    draw_object(scenario)
    rrt.draw()
    plt.gca().set_aspect('equal')

    # avoid collisions between ego vehicles, therefore add found
    # trajectory to the collision checker
    if p.has_solution():
        collision_checker.add_collision_object(p.car.create_collision_object())

plt.show()

# plt.figure(figsize=(25, 10))
# draw_object(scenario)
# draw_object(planning_task)
# plt.gca().set_aspect('equal')
# plt.show()