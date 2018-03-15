import xml.etree.ElementTree as et
from xml.dom import minidom
import pathlib
import numpy as np
import pyfvks
import fvks.scenario
import datetime

from .util import Point, Pointlist, ExactValue, UncertainIntervalScalar
from fvks.scenario.obstacle import ObstacleRole, ObstacleType
from fvks.scenario.trajectory import StateTupleFactory
import fvks.scenario

# TODO:
# - uncertain states


class CommonRoadFileWriter:

    def __init__(self, scenario, planning_task):
        self.scenario = scenario
        self.planning_task = planning_task
        self._root_node = et.Element('commonRoad')

    def _write_header(self):
        self._root_node.set('timeStepSize', str(self.scenario.dt))
        self._root_node.set('commonRoadVersion', '2017a')
        try:
            if self.scenario.benchmark_id:
                self._root_node.set('benchmarkID', self.scenario.benchmark_id)
        except:
            self._root_node.set('benchmarkID', '-1')
            print('Warning: No benchmark id set.')

        self._root_node.set('date', 
                            datetime.datetime.today().strftime('%d-%b-%Y'))

    def _add_all_objects_from_scenario(self):
        for l in self.scenario.lanelet_network.lanelets:
            self._root_node.append(LaneletXMLNode.create_node(l))
        for o in self.scenario.get_obstacles_by_role_and_type():
            self._root_node.append(ObstacleXMLNode.create_node(
                o, self.scenario.dt))

    def _add_all_planning_problems_from_planning_task(self):
        for planning_problem in self.planning_task.planning_problems:
            self._root_node.append(PlanningProblemXMLNode.create_node(
                planning_problem, self.scenario.dt))

    def _dump(self):
        rough_string = et.tostring(self._root_node, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

    def write_to_file(self, filename):
        if pathlib.Path(filename).is_file():
            print("File already exist. Aborting.")
            return
        file = open(filename, "w")
        self._write_header()
        self._add_all_objects_from_scenario()
        self._add_all_planning_problems_from_planning_task()
        file.write(self._dump())
        file.close()

    def write_scenario_to_file(self, filename):
        if pathlib.Path(filename).is_file():
            print("File already exist. Aborting.")
            return
        file = open(filename, "w")
        self._write_header()
        self._add_all_objects_from_scenario()
        file.write(self._dump())
        file.close()


class LaneletXMLNode:

    @classmethod
    def create_node(cls, lanelet):
        lanelet_node = et.Element('lanelet')
        lanelet_node.set('id', str(lanelet.lanelet_id))

        left_boundary = et.Element('leftBound')
        Pointlist.create_from_numpy_array(lanelet.left_vertices)\
                 .add_points_to_node(left_boundary)
        lanelet_node.append(left_boundary)

        right_boundary = et.Element('rightBound')
        Pointlist.create_from_numpy_array(lanelet.right_vertices)\
                 .add_points_to_node(right_boundary)
        lanelet_node.append(right_boundary)

        for l in lanelet.predecessor:
            predecessor = et.Element('predecessor')
            predecessor.set('ref', str(l))
            lanelet_node.append(predecessor)

        for l in lanelet.successor:
            successor = et.Element('successor')
            successor.set('ref', str(l))
            lanelet_node.append(successor)

        if lanelet.adj_left:
            adjacent_left = et.Element('adjacentLeft')
            adjacent_left.set('ref', str(lanelet.adj_left))
            if lanelet.adj_left_same_direction:
                adjacent_left.set('drivingDir', 'same')
            else:
                adjacent_left.set('drivingDir', 'opposite')
            lanelet_node.append(adjacent_left)

        if lanelet.adj_right:
            adjacent_right = et.Element('adjacentRight')
            adjacent_right.set('ref', str(lanelet.adj_right))
            if lanelet.adj_right_same_direction:
                adjacent_right.set('drivingDir', 'same')
            else:
                adjacent_right.set('drivingDir', 'opposite')
            lanelet_node.append(adjacent_right)

        if lanelet.speed_limit:
            speed_limit = et.Element('speedLimit')
            speed_limit.text = str(lanelet.speed_limit)
            lanelet_node.append(speed_limit)

        return lanelet_node


class ObstacleXMLNode:

    @classmethod
    def create_node(cls, obstacle, dt):
        if type(obstacle) == fvks.scenario.SimulatedCar:
            return SimulatedCarXMLNode.create_node(obstacle)
        elif type(obstacle) == fvks.scenario.ObstacleContainer:
            if (type(obstacle.stored_object) ==
                    pyfvks.collision.TimeVariantCollisionObject):
                return TimevariantObstacleXMLNode.create_node(obstacle, dt)
            elif (type(obstacle.stored_object) ==
                    pyfvks.collision.ShapeGroup or
                  type(obstacle.stored_object) ==
                    pyfvks.collision.RectOBB or
                  type(obstacle.stored_object) ==
                    pyfvks.collision.RectAABB):
                return StaticObstacleXMLNode.create_node(obstacle)
        else:
            raise Exception()

    @classmethod
    def _obstacle_type_enum_to_string(cls, obstacle_type):
        if obstacle_type == ObstacleType.car:
            return 'car'
        elif obstacle_type == ObstacleType.parkedVehicle:
            return 'parkedVehicle'
        elif obstacle_type == ObstacleType.unknown:
            return 'unknown'

    @classmethod
    def _obstacle_role_enum_to_string(cls, obstacle_role):
        if obstacle_role == ObstacleRole.static:
            return 'static'
        elif obstacle_role == ObstacleRole.dynamic:
            return 'dynamic'

    @classmethod
    def create_obstacle_node_header(cls, obstacle_id, obstacle_role,
                                    obstacle_type):
        obstacle_node = et.Element('obstacle')
        obstacle_node.set('id', str(obstacle_id))
        role_node = et.Element('role')
        role_node.text = cls._obstacle_role_enum_to_string(obstacle_role)
        obstacle_node.append(role_node)
        type_node = et.Element('type')
        type_node.text = cls._obstacle_type_enum_to_string(obstacle_type)
        obstacle_node.append(type_node)
        return obstacle_node


class StaticObstacleXMLNode:
    @classmethod
    def create_node(cls, obstacle):
        node = ObstacleXMLNode.create_obstacle_node_header(
            obstacle.obstacle_id, obstacle.obstacle_role,
            obstacle.obstacle_type)
        shape_node = et.Element('shape')
        shape_node.append(ShapeXMLNode.create_node(obstacle.stored_object))
        node.append(shape_node)
        return node


class SimulatedCarXMLNode:

    @classmethod
    def create_node(cls, car):
        obstacle_node = ObstacleXMLNode.create_obstacle_node_header(
            car.obstacle_id, car.obstacle_role, car.obstacle_type)
        obstacle_node.append(cls._create_shape_node(car))
        obstacle_node.append(cls._create_trajectory_node(car.trajectory,
                                                         car.dt))
        return obstacle_node

    @classmethod
    def _create_shape_node(cls, car):

        shape_node = et.Element('shape')
        rect_node = et.Element('rectangle')
        length_node = et.Element('length')
        length_node.text = str(car.length)
        rect_node.append(length_node)
        width_node = et.Element('width')
        width_node.text = str(car.width)
        rect_node.append(width_node)
        shape_node.append(rect_node)
        return shape_node

    @classmethod
    def _create_trajectory_node(cls, trajectory, dt):
        traj_node = et.Element('trajectory')
        state_time = trajectory.t0*dt
        for state in trajectory.state_list:
            state_node = et.Element('state')
            traj_node.append(StateXMLNode.create_state_node(state, state_node, 
                                                            state_time))
            state_time += dt
        return traj_node


class TimevariantObstacleXMLNode:

    @classmethod
    def create_node(cls, obstacle, dt):
        node = ObstacleXMLNode.create_obstacle_node_header(
            obstacle.obstacle_id, obstacle.obstacle_role,
            obstacle.obstacle_type)
        node.append(cls._create_occupancy_set(obstacle.stored_object, dt))
        return node

    @classmethod
    def _create_occupancy_set(cls, tvo, dt):
        occupancy_set = et.Element('occupancySet')
        for time_idx in range(tvo.time_start_idx(), tvo.time_end_idx() + 1):
            occupancy_set.append(cls._create_occupancy_set_at_time(tvo,
                                                                   time_idx,
                                                                   dt))
        return occupancy_set

    @classmethod
    def _create_occupancy_set_at_time(cls, tvo, time_idx, dt):
        occupancy = et.Element('occupancy')
        shape_node = et.Element('shape')
        obstacles_at_time = ShapeXMLNode.create_node(
            tvo.get_obstacle_at_time(time_idx))
        if type(obstacles_at_time) == list:
            for o in obstacles_at_time:
                shape_node.append(o)
        else:
            shape_node.append(obstacles_at_time)
        occupancy.append(shape_node)
        time = time_idx * dt
        time_node = et.Element('time')
        time_node.append(ExactValue(time).create_node())
        occupancy.append(time_node)
        return occupancy


class ShapeXMLNode:

    @classmethod
    def create_node(cls, shape):
        #shape_node = et.Element('shape')
        if type(shape) == pyfvks.collision.ShapeGroup:
            shape_node = []
            for s in shape.unpack():
                shape_node.append(cls._create_single_element(s))
        else:
            shape_node = cls._create_single_element(shape)
        return shape_node

    @classmethod
    def _create_single_element(cls, shape):
        if type(shape) == pyfvks.collision.Point:
            node = cls._create_pyfvks_collision_point_node(shape)
        elif type(shape) == pyfvks.collision.RectAABB:
            node = cls._create_pyfvks_collision_rectangleaabb_node(shape)
        elif type(shape) == pyfvks.collision.RectOBB:
            node =  cls._create_pyfvks_collision_rectangleobb_node(shape)
        elif type(shape) == pyfvks.collision.Circle:
            node = cls._create_pyfvks_collision_circle_node(shape)
        elif type(shape) == pyfvks.collision.Triangle:
            node = cls._create_pyfvks_collision_triangle_node(shape)
        else:
            raise Exception()
        return node

    @classmethod
    def _create_pyfvks_collision_point_node(cls, point):
        point_node = et.Element('point')
        x = et.Element('x')
        x.text = str(np.float64(point.center()[0]))
        point_node.append(x)
        y = et.Element('y')
        y.text = str(np.float64(point.center()[1]))
        point_node.append(y)
        return point_node

    @classmethod
    def _create_pyfvks_collision_rectangleaabb_node(cls, aabb):
        length = aabb.max_x() - aabb.min_x()
        width = aabb.max_y() - aabb.min_y()
        orientation = 0.0
        x = (aabb.max_x() + aabb.min_x())/2.0
        y = (aabb.max_y() + aabb.min_y())/2.0

        rectangle_node = et.Element('rectangle')

        length_node = et.Element('length')
        length_node.text = str(length)
        rectangle_node.append(length_node)

        width_node = et.Element('width')
        width_node.text = str(width)
        rectangle_node.append(width_node)

        orientation_node = et.Element('orientation')
        orientation_node.text = str(np.float64(orientation))
        rectangle_node.append(orientation_node)

        center_node = et.Element('center')
        x_node = et.Element('x')
        x_node.text = str(np.float64(x))
        center_node.append(x_node)
        y_node = et.Element('y')
        y_node.text = str(np.float64(y))
        center_node.append(y_node)
        rectangle_node.append(center_node)

        return rectangle_node

    @classmethod
    def _create_pyfvks_collision_rectangleobb_node(cls, obb):
        length = 2 * obb.r_x()
        width = 2 * obb.r_y()
        orientation = np.arctan2(obb.local_x_axis()[1],
                                 obb.local_x_axis()[0])
        x = obb.center()[0]
        y = obb.center()[1]

        rectangle_node = et.Element('rectangle')

        length_node = et.Element('length')
        length_node.text = str(length)
        rectangle_node.append(length_node)

        width_node = et.Element('width')
        width_node.text = str(width)
        rectangle_node.append(width_node)

        orientation_node = et.Element('orientation')
        orientation_node.text = str(np.float64(orientation))
        rectangle_node.append(orientation_node)

        center_node = et.Element('center')
        x_node = et.Element('x')
        x_node.text = str(np.float64(x))
        center_node.append(x_node)
        y_node = et.Element('y')
        y_node.text = str(np.float64(y))
        center_node.append(y_node)
        rectangle_node.append(center_node)

        return rectangle_node

    @classmethod
    def _create_pyfvks_collision_circle_node(cls, circle):
        radius = circle.r()
        x = circle.x()
        y = circle.y()

        circle_node = et.Element('circle')

        radius_node = et.Element('radius')
        radius_node.text = str(np.float64(radius))
        circle_node.append(radius_node)

        center_node = et.Element('center')
        x_node = et.Element('x')
        x_node.text = str(np.float64(x))
        center_node.append(x_node)
        y_node = et.Element('y')
        y_node.text = str(np.float64(y))
        center_node.append(y_node)
        circle_node.append(center_node)

        return circle_node

    @classmethod
    def _create_pyfvks_collision_triangle_node(cls, triangle):
        triangle_node = et.Element('polygon')
        for v in triangle.vertices():
            triangle_node.append(Point(v[0], v[1]).create_node())
        return triangle_node


class StateXMLNode:
    @classmethod
    def create_goal_node(cls, state, time_step):
        state_node = et.Element('state')
        if StateTupleFactory.position in state._fields:
            position = et.Element('position')
            position = cls._write_goal_position(position, state.position)
            state_node.append(position)
        if StateTupleFactory.orientation in state._fields:
            orientation = et.Element('orientation')
            orientation = cls._write_goal_value_exact_or_interval(
                orientation, state.orientation)
            state_node.append(orientation)
        if StateTupleFactory.time in state._fields:
            time = et.Element('time')
            time = cls._write_goal_time_exact_or_interval(time, state.time, time_step)
            state_node.append(time)
        if StateTupleFactory.velocity in state._fields:
            velocity = et.Element('velocity')
            velocity = cls._write_goal_value_exact_or_interval(
                velocity, state.velocity)
            state_node.append(velocity)
        if StateTupleFactory.acceleration in state._fields:
            acceleration = et.Element('acceleration')
            acceleration = cls._write_goal_value_exact_or_interval(
                acceleration, state.acceleration)
            state_node.append(acceleration)
        if StateTupleFactory.yawRate in state._fields:
            yaw_rate = et.Element('yawRate')
            yaw_rate = cls._write_goal_value_exact_or_interval(
                yaw_rate, state.yawRate)
            state_node.append(yaw_rate)
        if StateTupleFactory.slipAngle in state._fields:
            slip_angle = et.Element('slipAngle')
            slip_angle = cls._write_goal_value_exact_or_interval(
                slip_angle, state.slipAngle)
            state_node.append(slip_angle)
        return state_node

    @classmethod
    def _write_goal_position(cls, node, position):
        if (type(position) == pyfvks.collision.Point or
            type(position) == pyfvks.collision.RectOBB or
            type(position) == pyfvks.collision.Circle or
            type(position) == pyfvks.collision.Triangle):
            node.append(ShapeXMLNode.create_node(position))
        elif type(position) == pyfvks.collision.ShapeGroup:
            node.extend(ShapeXMLNode.create_node(position))
        elif type(position) == fvks.scenario.Lanelet:
            lanelet = et.Element('lanelet')
            lanelet.set('ref', str(position.lanelet_id))
            node.append(lanelet)
        elif type(position) is list:
            for p in position:
                node = cls._write_goal_position(node, p)
        else:
            raise Exception()
        return node

    @classmethod
    def _write_goal_time_exact_or_interval(cls, node, var, time_step):
        if isinstance(var, ExactValue):
            node.append(ExactValue(var.value*time_step).create_node())
        elif isinstance(var, UncertainIntervalScalar):
            node.extend(UncertainIntervalScalar(var.value_lo*time_step, var.value_hi*time_step).create_node())
        else:
            raise Exception()
        return node

    @classmethod
    def _write_goal_value_exact_or_interval(cls, node, var):
        if isinstance(var, ExactValue):
            node.append(ExactValue(var.value).create_node())
        elif isinstance(var, UncertainIntervalScalar):
            node.extend(UncertainIntervalScalar(var.value_lo, var.value_hi).create_node())
        else:
            raise Exception()
        return node


    @classmethod
    def create_state_node(cls, state, state_node, time):
        if StateTupleFactory.position in state._fields:
            position = et.Element('position')
            if type(state.position) == np.ndarray:
                position.append(Point.create_from_numpy_array(state.position)
                                     .create_node())
                state_node.append(position)
            else:
                raise Exception()
        if StateTupleFactory.orientation in state._fields:
            orientation = et.Element('orientation')
            orientation.append(ExactValue(state.orientation).create_node())
            state_node.append(orientation)

        time_node = et.Element('time')
        time_node.append(ExactValue(time).create_node())
        state_node.append(time_node)

        if StateTupleFactory.velocity in state._fields:
            velocity = et.Element('velocity')
            velocity.append(ExactValue(state.velocity).create_node())
            state_node.append(velocity)
        if StateTupleFactory.acceleration in state._fields:
            acceleration = et.Element('acceleration')
            acceleration.append(ExactValue(state.acceleration).create_node())
            state_node.append(acceleration)
        if StateTupleFactory.yawRate in state._fields:
            yaw_rate = et.Element('yawRate')
            yaw_rate.append(ExactValue(state.yawRate).create_node())
            state_node.append(yaw_rate)
        if StateTupleFactory.slipAngle in state._fields:
            slip_angle = et.Element('slipAngle')
            slip_angle.append(ExactValue(state.slipAngle).create_node())
            state_node.append(slip_angle)

        return state_node


class PlanningProblemXMLNode:
    @classmethod
    def create_node(cls, planning_problem, time_step):
        planning_problem_node = et.Element('planningProblem')
        planning_problem_node.set('id',
                                  str(planning_problem.planning_problem_id))
        initial_state_node = et.Element('initialState')
        planning_problem_node.append(StateXMLNode.create_state_node(
            planning_problem.initial_state, initial_state_node, 
            planning_problem.initial_state.time*time_step))
        goal_node = et.Element('goalRegion')
        for goal_state in planning_problem.goal.state_list:
            goal_node.append(StateXMLNode.create_goal_node(goal_state, time_step))
        planning_problem_node.append(goal_node)
        return planning_problem_node
