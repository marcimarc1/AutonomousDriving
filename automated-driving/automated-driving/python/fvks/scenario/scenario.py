import textwrap
import pyfvks

from .scenario_exception import ScenarioError
from .lanelet import LaneletNetwork
from .lanelet import Lanelet
from .traffic import SimulatedCar
from .obstacle import ObstacleRole
from .obstacle import ObstacleType
from .obstacle import ObstacleContainer


class Scenario(object):

    def __init__(self, dt, benchmark_id=None):
        self.dt = dt
        self.lanelet_network = LaneletNetwork()
        #self.vehicles = []
        self._obstacles = []
        self.current_time = 0
        self._id_set = set()
        self._id_count = 1000
        self.benchmark_id = benchmark_id

    def _is_object_id_used(self, object_id):
        return object_id in self._id_set

    def _mark_object_id_as_used(self, object_id):
        if self._is_object_id_used(object_id):
            raise Exception()
        self._id_set.add(object_id)

    def generate_object_id(self):
        while self._is_object_id_used(self._id_count):
            self._id_count += 1
        return self._id_count

    def _add_static_unknown_obstacle(self, o):
        self._check_id_and_add_obstacle(ObstacleContainer(
            self.generate_object_id(), ObstacleRole.static,
            ObstacleType.unknown, o))

    def _check_id_and_add_obstacle(self, o):
        if self._is_object_id_used(o.obstacle_id):
            print("Id {} is already used".format(o.obstacle_id))
            raise Exception()
        self._mark_object_id_as_used(o.obstacle_id)
        self._obstacles.append(o)

    def add_objects(self, o):
        if type(o) == list:
            for oo in o:
                self.add_objects(oo)
        elif type(o) == ObstacleContainer:
            self._check_id_and_add_obstacle(o)
        elif (type(o) == pyfvks.collision.RectOBB or
              type(o) == pyfvks.collision.RectAABB or
              type(o) == pyfvks.collision.ShapeGroup or
              type(o) == pyfvks.collision.Triangle):
            print('deprecated: do not add pyfvks.collision.* objects directly')
            self._add_static_unknown_obstacle(o)
        elif type(o) == pyfvks.collision.TimeVariantCollisionObject:
            self._check_id_and_add_obstacle(ObstacleContainer(
                self.generate_object_id(), ObstacleRole.dynamic,
                ObstacleType.unknown, o))
        elif type(o) == SimulatedCar:
            if o.dt != self.dt:
                raise ScenarioError()
            self._check_id_and_add_obstacle(o)
        elif type(o) == LaneletNetwork:
            for l in o.lanelets:
                self._mark_object_id_as_used(l.lanelet_id)
            self.lanelet_network.add_lanelet_network(o)
        elif type(o) == Lanelet:
            self._mark_object_id_as_used(o.lanelet_id)
            self.lanelet_network.add_lanelet(o)
        else:
            print(type(o))
            print(o)
            assert False

    def create_collision_checker(self):
        cc = pyfvks.collision.CollisionChecker()
        for co in self._obstacles:
            cc.add_collision_object(co.create_collision_object())
        return cc

    def get_obstacles_by_role_and_type(self, obstacle_role=None,
                                       obstacle_type=None):
        result = list()
        for o in self._obstacles:
            if ((obstacle_role is None or o.obstacle_role == obstacle_role) and
                (obstacle_type is None or o.obstacle_type == obstacle_type)):
                result.append(o)
        return result

    def get_obstacle_by_id(self, obstacle_id):
        if type(obstacle_id) == list:
            if len(obstacle_id) == 0:
                return None
            obstacles = list()
            for o in obstacle_id:
                obstacles.append(self.get_obstacle_by_id(o))
            return obstacles
        else:
            for o in self._obstacles:
                if o.obstacle_id == obstacle_id:
                    return o
        return None

    def _get_by_object_by_id(self, object_id):
        tmp = self.get_obstacle_by_id(object_id)
        if tmp is not None:
            return tmp
        # TODO: Add lanelet

    def get_active_traffic(self, window=None):
        """

        :param window: [x_min x_max y_min y_max]
        :return:
        """
        car_list = [o for o in
                    self.get_obstacles_by_role_and_type(ObstacleRole.dynamic,
                                                        ObstacleType.car)]
        query = list()
        for t in car_list:
            if t.active:
                if not window:
                    query.append(t)
                else:
                    if ((window[0] <= t.position[0] <= window[1]) and
                            (window[2] <= t.position[1] <= window[3])):
                        query.append(t)
        return query

    def get_traffic_by_car_id(self, car_id):
        car_list = [o.obstacle_object for o in
                    self.get_obstacles_by_role_and_type(ObstacleRole.dynamic,
                                                        ObstacleType.car)]
        for t in car_list:
            if t.obstacle_id == car_id:
                return t
        return None

    def get_time_variant_obstacles(self):
        tvo_list = [o.obstacle_object for o in
                    self.get_obstacles_by_role_and_type(ObstacleRole.dynamic,
                                                        ObstacleType.unknown)]
        for tvo in tvo_list:
            assert type(tvo) == pyfvks.collision.TimeVariantCollisionObject
        return tvo_list

    def get_static_obstacles(self):
        return [o.stored_object for o in
                self.get_obstacles_by_role_and_type(ObstacleRole.static)]

    def set_time(self, time):
        self.current_time = time

        update_list = self.get_obstacles_by_role_and_type(ObstacleRole.dynamic,
                                                          ObstacleType.car)
        for cars in update_list:
            try:
                cars.update_time(self.current_time)
            except AttributeError:
                pass


    def step(self):
        self.current_time += 1
        self.set_time(self.current_time)

    def _get_timespan(self):
        raise Exception()
        if self.timevariant_obstacles:
            time_span_start = self.timevariant_obstacles[0].time_start_idx()
            time_span_end = self.timevariant_obstacles[0].time_end_idx()
            for tvo in self.timevariant_obstacles:
                if tvo.time_end_idx() >= tvo.time_start_idx():
                    time_span_start = min(time_span_start, tvo.time_start_idx())
                    time_span_end = max(time_span_end, tvo.time_end_idx())
            return time_span_start, time_span_end
        return None

    def translate_rotate(self, translation, angle):
        self.lanelet_network.translate_rotate(translation, angle)
        for o in self._obstacles:
            o.translate_rotate(translation, angle)
        self.set_time(self.current_time)

    def __str__(self):

        def table_entry(col_a, col_b, spacing):
            col_b_wrapped = textwrap.wrap(str(col_b), 80-spacing)

            ret_str = ("{:" + str(spacing) + "}{}\n").format(
                col_a, col_b_wrapped[0])
            for l in col_b_wrapped[1:]:
                ret_str += (
                    "{:" + str(spacing) + "}{}\n").format('', l)
            return ret_str

        spacing = 15

        h_line = ("----------------------------------------"
                  "---------------------------------------")

        traffic_str = h_line + "\n"
        traffic_str += "Scenario\n"
        traffic_str += h_line + "\n\n"
        traffic_str += table_entry("Benchmark ID:", self.benchmark_id, spacing)
        traffic_str += table_entry("Current time:", self.current_time, spacing)
        traffic_str += table_entry("Timestep:", self.dt, spacing)
        traffic_str += "\nObstacles:\n"
        traffic_str += h_line + "\n"
        for o in self._obstacles:
            traffic_str += table_entry('{:8d}'.format(o.obstacle_id),
                                       'obstacle ({}, {}, {})'.format(
                                           o.obstacle_role,
                                           o.obstacle_type,
                                           str(type(o))),
                                       spacing)
        traffic_str += "\nLanelets:\n"
        traffic_str += h_line + "\n"
        traffic_str += str(self.lanelet_network)
        return traffic_str
