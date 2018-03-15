import enum
import numpy as np

import pyfvks
import fvks.geometry.transform


class ObstacleRole(enum.Enum):
    static = 1
    dynamic = 2


class ObstacleType(enum.Enum):
    car = 1
    unknown = 2
    prediction = 3
    parkedVehicle = 4


class Obstacle:
    def __init__(self, obstacle_id, obstacle_role, obstacle_type):
        self.obstacle_id = obstacle_id
        self.obstacle_role = obstacle_role
        self.obstacle_type = obstacle_type


class ObstacleContainer(Obstacle):
    def __init__(self, obstacle_id, obstacle_role, obstacle_type,
                 stored_object):
        Obstacle.__init__(self, obstacle_id, obstacle_role, obstacle_type)
        self.stored_object = stored_object

    def create_collision_object(self):
        if type(self.stored_object) in [
             pyfvks.collision.RectOBB, pyfvks.collision.Triangle,
             pyfvks.collision.TimeVariantCollisionObject,
             pyfvks.collision.ShapeGroup]:
            return self.stored_object
        else:
            print(self.stored_object)
            raise Exception()

    def translate_rotate(self, translation, angle):
        if type(self.stored_object) == pyfvks.collision.RectOBB:
            o = self.stored_object
            t_m = fvks.geometry.transform.translation_rotation_matrix(
                translation, angle)
            new_position = t_m.dot(
                np.array([[o.center()[0]], [o.center()[1]], [1]]))
            self.stored_object = pyfvks.collision.RectOBB(
                o.r_x(), o.r_y(),
                np.arctan2(o.local_x_axis()[1], o.local_x_axis()[0]) + angle,
                new_position[0],
                new_position[1])
        else:
            raise Exception()
