import xml.etree.ElementTree as et
import numpy as np
import pyfvks
import fvks.scenario

from .util import Point, Pointlist, ExactValue, UncertainIntervalScalar
from fvks.scenario.obstacle import ObstacleRole, ObstacleType
from fvks.scenario.trajectory import StateTupleFactory
from fvks.scenario.trajectory import Trajectory
import fvks.scenario
import fvks.planning.planning

# TODO:
# - uncertain states


class CommonRoadFileReader:

    def __init__(self, filename):
        self._filename = filename
        self._tree = None
        self._time_step = None
        self._benchmark_id = None
        self._scenario = None
        self._planning_task = None

    def open(self):
        self.open_scenario()
        self._planning_task = PlanningTaskFactory.create_from_xml_node(self._tree, self._scenario)
        return self._scenario, self._planning_task

    def open_scenario(self):
        self._parse_file()
        self._time_step = self._get_time_step()
        self._benchmark_id = self._get_benchmark_id()
        self._scenario = fvks.scenario.Scenario(self._time_step, self._benchmark_id)
        self._add_lanelets()
        self._add_obstacles()
        self._scenario.set_time(0)
        return self._scenario

    def _parse_file(self):
        self._tree = et.parse(self._filename)

    def _get_time_step(self):
        return float(self._tree.getroot().get('timeStepSize'))

    def _get_benchmark_id(self):
        return self._tree.getroot().get('benchmarkID')

    def _add_obstacles(self):
        for o in self._tree.findall('obstacle'):
            self._scenario.add_objects(
                ObstacleFactory.create_from_xml_node(o, self._time_step))

    def _add_lanelets(self):
        for lanelet_node in self._tree.findall('lanelet'):
            self._scenario.add_objects(LaneletFactory.create_from_xml_node(
                lanelet_node))


class LaneletFactory:

    @classmethod
    def create_from_xml_node(cls, xml_node):
        lanelet_id = int(xml_node.get('id'))
        left_bound = Pointlist.create_from_xml_node(
            xml_node.find('leftBound')).as_numpy_array()
        right_bound = Pointlist.create_from_xml_node(
            xml_node.find('rightBound')).as_numpy_array()
        center_vertices = 0.5 * (left_bound + right_bound)
        predecessors = []
        for l in xml_node.findall('predecessor'):
            predecessors.append(int(l.get('ref')))
        successors = []
        for l in xml_node.findall('successor'):
            successors.append(int(l.get('ref')))

        if xml_node.find('adjacentLeft') is not None:
            adjacent_left = int(xml_node.find('adjacentLeft').get('ref'))
            if xml_node.find('adjacentLeft').get('drivingDir') == 'same':
                adjacent_left_same_direction = True
            else:
                adjacent_left_same_direction = False
        else:
            adjacent_left = None
            adjacent_left_same_direction = None

        if xml_node.find('adjacentRight') is not None:
            adjacent_right = int(xml_node.find('adjacentRight').get('ref'))
            if xml_node.find('adjacentRight').get('drivingDir') == 'same':
                adjacent_right_same_direction = True
            else:
                adjacent_right_same_direction = False
        else:
            adjacent_right = None
            adjacent_right_same_direction = None

        return fvks.scenario.Lanelet(
            left_vertices=left_bound, center_vertices=center_vertices,
            right_vertices=right_bound, predecessor=predecessors,
            successor=successors, adjacent_left=adjacent_left,
            adjacent_left_same_direction=adjacent_left_same_direction,
            adjacent_right=adjacent_right,
            adjacent_right_same_direction=adjacent_right_same_direction,
            lanelet_id=lanelet_id, speed_limit=None)


class ObstacleFactory:

    @classmethod
    def create_from_xml_node(cls, xml_node, time_step):
        if xml_node.find('role').text == 'static':
            return StaticObstacleFactory.create_from_xml_node(xml_node)
        elif xml_node.find('role').text == 'dynamic':
            if xml_node.find('trajectory') is not None:
                return SimulatedCarFactory.create_from_xml_node(xml_node,
                                                                time_step)
            elif xml_node.find('occupancySet') is not None:
                return TimevariantObstacleFactory.create_from_xml_node(
                    xml_node, time_step)
            else:
                raise Exception()
        else:
            raise Exception()

    @classmethod
    def read_role(cls, xml_node):
        obstacle_role = cls._string_to_obstacle_role_enum(
            xml_node.find('role').text)
        return obstacle_role

    @classmethod
    def read_type(cls, xml_node):
        obstacle_type = cls._string_to_obstacle_type_enum(
            xml_node.find('type').text)
        return obstacle_type

    @classmethod
    def read_id(cls, xml_node):
        obstacle_id = int(xml_node.get('id'))
        return obstacle_id

    @classmethod
    def _string_to_obstacle_role_enum(cls, obstacle_role):
        if obstacle_role == 'static':
            return ObstacleRole.static
        elif obstacle_role == 'dynamic':
            return ObstacleRole.dynamic

    @classmethod
    def _string_to_obstacle_type_enum(cls, obstacle_type):
        if obstacle_type == 'car':
            return ObstacleType.car
        elif obstacle_type == 'parkedVehicle':
            return ObstacleType.parkedVehicle
        elif obstacle_type == 'unknown':
            return ObstacleType.unknown


class StaticObstacleFactory:

    @classmethod
    def create_from_xml_node(cls, xml_node):
        obstacle_role = ObstacleFactory.read_role(xml_node)
        obstacle_type = ObstacleFactory.read_type(xml_node)
        obstacle_id = ObstacleFactory.read_id(xml_node)
        shape = ShapeFactory.create_from_xml_node(xml_node.find('shape'))
        return fvks.scenario.ObstacleContainer(obstacle_id, obstacle_role,
                                               obstacle_type, shape)


class SimulatedCarFactory:

    # @classmethod
    # def _inspect_state_field_and_create_state_tuple(cls, xml_node):
    #     tuple_field_list = list()
    #     found_xml_fields = list()
    #     if xml_node.find('position') is not None:
    #         tuple_field_list.append(StateTupleFactory.position)
    #         found_xml_fields.append('position')
    #     if xml_node.find('orientation') is not None:
    #         tuple_field_list.append(StateTupleFactory.orientation)
    #         found_xml_fields.append('orientation')
    #     if xml_node.find('velocity') is not None:
    #         tuple_field_list.append(StateTupleFactory.velocity)
    #         found_xml_fields.append('velocity')
    #     if xml_node.find('acceleration') is not None:
    #         tuple_field_list.append(StateTupleFactory.acceleration)
    #         found_xml_fields.append('acceleration')
    #     time_idx = float(xml_node.find('time').find('exact').text)
    #     return (StateTupleFactory.create_state_tuple(*tuple_field_list),
    #             found_xml_fields, time_idx)
    #
    # @classmethod
    # def _read_state(cls, xml_node, state_tuple, required_xml_field):
    #     for check_xml_field in required_xml_field:
    #         if xml_node.find(check_xml_field) is None:
    #             raise Exception()
    #
    #     point = Point.create_from_xml_node(xml_node.find('position')
    #                                                .find('point'))\
    #                  .as_numpy_array()
    #     orientation = float(xml_node.find('orientation').find('exact').text)
    #     time = float(xml_node.find('time').find('exact').text)
    #     state_args = {'position': point, 'orientation': orientation}
    #     if xml_node.find('velocity') is not None:
    #         speed = float(xml_node.find('velocity').find('exact').text)
    #         state_args['velocity'] = speed
    #     if xml_node.find('acceleration') is not None:
    #         speed = float(xml_node.find('acceleration').find('exact').text)
    #         state_args['acceleration'] = speed
    #     state = state_tuple(**state_args)
    #     return state, time
    #
    @classmethod
    def _read_trajectory(cls, xml_node, time_step):
        state_tuple, xml_fields = StateFactory.inspect_state_field_and_create_state_tuple(xml_node.find('state'))
        time_0 = float(xml_node.find('state').find('time').find('exact').text)
        time_idx_0 = int(round(time_0 / time_step))
        state_list = list()
        for state_node in xml_node.findall('state'):
            state_list.append(StateFactory.read_state(
                state_node, state_tuple, xml_fields, time_step))
        return Trajectory(time_idx_0, state_list, state_tuple)

    @classmethod
    def create_from_xml_node(cls, xml_node, time_step):
        obstacle_role = ObstacleFactory.read_role(xml_node)
        obstacle_type = ObstacleFactory.read_role(xml_node)
        obstacle_id = ObstacleFactory.read_id(xml_node)
        trajectory = cls._read_trajectory(xml_node.find('trajectory'),
                                          time_step)
        rectangle_node = xml_node.find('shape').find('rectangle')
        length = float(rectangle_node.find('length').text)
        width = float(rectangle_node.find('width').text)
        return fvks.scenario.SimulatedCar(trajectory, time_step, obstacle_id,
                                          length, width)


class TimevariantObstacleFactory:

    @classmethod
    def create_from_xml_node(cls, xml_node, time_step):
        obstacle_role = ObstacleFactory.read_role(xml_node)
        obstacle_type = ObstacleFactory.read_type(xml_node)
        obstacle_id = ObstacleFactory.read_id(xml_node)
        time_variant_obstacle = cls._read_occupancy_set(xml_node.find(
            'occupancySet'), time_step)
        return fvks.scenario.ObstacleContainer(obstacle_id, obstacle_role,
                                               obstacle_type,
                                               time_variant_obstacle)

    @classmethod
    def _read_occupancy_set(cls, xml_node, time_step):
        time = float(xml_node.find('occupancy').find('time').find('exact').text)
        time_idx_0 = int(round(time / time_step))
        tvo = pyfvks.collision.TimeVariantCollisionObject(time_idx_0)
        for occupancy in xml_node.findall('occupancy'):
            tvo.append_obstacle(cls._read_occupancy(occupancy)[0])
        return tvo

    @classmethod
    def _read_occupancy(cls, xml_node):
        shape = ShapeFactory.create_from_xml_node(xml_node.find('shape'))
        time = float(xml_node.find('time').find('exact').text)
        return shape, time


class ShapeFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node):
        shape_list = []
        for c in xml_node.getchildren():
            shape_list.append(cls._read_single_shape(c))
        shape = cls._create_shape_group_if_needed(shape_list)
        return shape

    @classmethod
    def _read_single_shape(cls, xml_node):
        tag_string = xml_node.tag
        if tag_string == 'point':
            return cls._read_point(xml_node)
        elif tag_string == 'rectangle':
            return cls._read_obb(xml_node)
        elif tag_string == 'circle':
            return cls._read_circle(xml_node)
        elif tag_string == 'polygon':
            return cls._read_polygon(xml_node)

    @classmethod
    def _read_point(cls, xml_node):
        x = float(xml_node.find('x').text)
        y = float(xml_node.find('y').text)
        return pyfvks.collision.Point(x, y)

    @classmethod
    def _read_obb(cls, xml_node):
        length = float(xml_node.find('length').text)
        width = float(xml_node.find('width').text)
        orientation = float(xml_node.find('orientation').text)
        x = float(xml_node.find('center').find('x').text)
        y = float(xml_node.find('center').find('y').text)
        return pyfvks.collision.RectOBB(length/2, width/2, orientation, x, y)

    @classmethod
    def _read_polygon(cls, xml_node):
        point_list = Pointlist.create_from_xml_node(xml_node)
        if len(point_list.points) == 3:
            point_list = point_list.as_numpy_array()
            return pyfvks.collision.Triangle(point_list[0][0], point_list[0][1],
                                             point_list[1][0], point_list[1][1],
                                             point_list[2][0], point_list[2][1])
        else:
            raise Exception()

    @classmethod
    def _read_circle(cls, xml_node):
        r = float(xml_node.find('radius').text)
        x = float(xml_node.find('center').find('x').text)
        y = float(xml_node.find('center').find('y').text)
        return pyfvks.collision.Circle(r, x, y)


    @classmethod
    def _create_shape_group_if_needed(cls, shape_list):
        if len(shape_list) > 1:
            sg = pyfvks.collision.ShapeGroup()
            for s in shape_list:
                if not (type(s) == pyfvks.collision.RectOBB or
                        type(s) == pyfvks.collision.Triangle or
                        type(s) == pyfvks.collision.Circle or
                        type(s) == pyfvks.collision.RectAAABB):
                    raise Exception()
                sg.add_shape(s)
            return sg
        else:
            return shape_list[0]


class PlanningTaskFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node, scenario):
        planning_task = fvks.planning.planning.PlanningTask()
        for p in xml_node.findall('planningProblem'):
            planning_task.add_planning_problem(PlanningProblemFactory.create_from_xml_node(p, scenario))
        return planning_task


class PlanningProblemFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node, scenario):
        planning_problem_id = int(xml_node.get('id'))
        initial_state = cls._add_initial_state(xml_node, scenario.dt)
        goals = cls._add_goal_region(xml_node, scenario)
        return fvks.planning.planning.PlanningProblem(planning_problem_id, initial_state, goals)

    @classmethod
    def _add_initial_state(cls, xml_node, time_step):
        state_tuple, xml_fields = StateFactory.inspect_state_field_and_create_state_tuple(xml_node.find('initialState'))
        initial_state = StateFactory.read_state(xml_node.find('initialState'), state_tuple, xml_fields, time_step)
        return initial_state

    @classmethod
    def _add_goal_region(cls, xml_node, scenario):
        goal_region = xml_node.findall('goalRegion')
        if len(goal_region) > 1:
            raise Exception()
        goal_state_list = list()
        for state_node in goal_region[0].findall('state'):
            state_tuple, xml_fields = StateFactory.inspect_state_field_and_create_state_tuple(state_node)
            goal_state_list.append(StateFactory.read_goal_state(state_node, state_tuple, xml_fields, scenario))
        return goal_state_list


class StateFactory:

    @classmethod
    def inspect_state_field_and_create_state_tuple(cls, xml_node):
        tuple_field_list = list()
        found_xml_fields = list()
        if xml_node.find('position') is not None:
            tuple_field_list.append(StateTupleFactory.position)
            found_xml_fields.append('position')
        if xml_node.find('orientation') is not None:
            tuple_field_list.append(StateTupleFactory.orientation)
            found_xml_fields.append('orientation')
        if xml_node.find('velocity') is not None:
            tuple_field_list.append(StateTupleFactory.velocity)
            found_xml_fields.append('velocity')
        if xml_node.find('acceleration') is not None:
            tuple_field_list.append(StateTupleFactory.acceleration)
            found_xml_fields.append('acceleration')
        if xml_node.find('time') is not None:
            tuple_field_list.append(StateTupleFactory.time)
            found_xml_fields.append('time')
        if xml_node.find('yawRate') is not None:
            tuple_field_list.append(StateTupleFactory.yawRate)
            found_xml_fields.append('yawRate')
        if xml_node.find('slipAngle') is not None:
            tuple_field_list.append(StateTupleFactory.slipAngle)
            found_xml_fields.append('slipAngle')
        return (StateTupleFactory.create_state_tuple(*tuple_field_list),
                found_xml_fields)

    @classmethod
    def read_goal_state(cls, xml_node, state_tuple, required_xml_field, scenario):
        for check_xml_field in required_xml_field:
            if xml_node.find(check_xml_field) is None:
                raise Exception()

        state_args = {}

        if xml_node.find('position') is not None:
            position = cls._read_goal_position(xml_node.find('position'), scenario)
            state_args['position'] = position
        if xml_node.find('orientation') is not None:
            orientation = cls._read_goal_value_exact_or_interval(xml_node.find('orientation'))
            state_args['orientation'] = orientation
        if xml_node.find('time') is not None:
            time = cls._read_goal_time_exact_or_interval(xml_node.find('time'), scenario.dt)
            state_args['time'] = time
        if xml_node.find('velocity') is not None:
            speed = cls._read_goal_value_exact_or_interval(xml_node.find('velocity'))
            state_args['velocity'] = speed
        if xml_node.find('acceleration') is not None:
            acceleration = cls._read_goal_value_exact_or_interval(xml_node.find('acceleration'))
            state_args['acceleration'] = acceleration
        if xml_node.find('yawRate') is not None:
            yaw_rate = cls._read_goal_value_exact_or_interval(xml_node.find('yawRate'))
            state_args['yawRate'] = yaw_rate
        if xml_node.find('yawRate') is not None:
            slip_angle = cls._read_goal_value_exact_or_interval(xml_node.find('slipAngle'))
            state_args['slipAngle'] = slip_angle
        state = state_tuple(**state_args)
        return state

    @classmethod
    def _read_goal_position(cls, xml_node, scenario):
        if(xml_node.find('point') is not None or
           xml_node.find('rectangle') is not None or
           xml_node.find('circle') is not None or
           xml_node.find('polygon') is not None):
            position = ShapeFactory.create_from_xml_node(xml_node)
        elif xml_node.find('lanelet') is not None:
            position = list()
            for l in xml_node.findall('lanelet'):
                 position.append(scenario.lanelet_network.find_lanelet_by_id(int(l.get('ref'))))
        else:
            raise Exception()
        return position

    @classmethod
    def _read_goal_time_exact_or_interval(cls, xml_node, time_step):
        if xml_node.find('exact') is not None:
            value = int(float(xml_node.find('time').find('exact').text)/time_step)
        elif xml_node.find('intervalStart') is not None and xml_node.find('intervalEnd') is not None:
            value = UncertainIntervalScalar(int(float(xml_node.find('intervalStart').text)/time_step),
                                            int(float(xml_node.find('intervalEnd').text)/time_step))
        else:
            raise Exception()
        return value

    @classmethod
    def _read_goal_value_exact_or_interval(cls, xml_node):
        if xml_node.find('exact') is not None:
            value = ExactValue(float(xml_node.find('exact').text))
        elif xml_node.find('intervalStart') is not None and xml_node.find('intervalEnd') is not None:
            value = UncertainIntervalScalar(float(xml_node.find('intervalStart').text),
                                            float(xml_node.find('intervalEnd').text))
        else:
            raise Exception()
        return value

    @classmethod
    def read_state(cls, xml_node, state_tuple, required_xml_field, time_step):
        for check_xml_field in required_xml_field:
            if xml_node.find(check_xml_field) is None:
                raise Exception()

        point = Point.create_from_xml_node(xml_node.find('position')
                                                   .find('point'))\
                     .as_numpy_array()
        orientation = float(xml_node.find('orientation').find('exact').text)
        time = int(float(xml_node.find('time').find('exact').text)/time_step)
        state_args = {'position': point, 'orientation': orientation, 'time': time}
        if xml_node.find('velocity') is not None:
            speed = float(xml_node.find('velocity').find('exact').text)
            state_args['velocity'] = speed
        if xml_node.find('acceleration') is not None:
            speed = float(xml_node.find('acceleration').find('exact').text)
            state_args['acceleration'] = speed
        if xml_node.find('yawRate') is not None:
            yaw_rate = float(xml_node.find('yawRate').find('exact').text)
            state_args['yawRate'] = yaw_rate
        if xml_node.find('slipAngle') is not None:
            slip_angle = float(xml_node.find('slipAngle').find('exact').text)
            state_args['slipAngle'] = slip_angle
        state = state_tuple(**state_args)
        return state
