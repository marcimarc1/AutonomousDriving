# class for the vehicle shape and maneuver area
import numpy as np
import math
from bertha.helpers.helpermethods import psi


class Vehicle:
    def __init__(self, circles, length, width, r_veh, kmax=None, amax=None, init_vel=None):
        self.circles_count = circles
        self.length = length
        self.width = width
        self.pos = []
        self.radius = r_veh
        self.kmax = kmax
        self.amax = amax
        self.init_vel = init_vel
        self.car_shape()
        if self.radius <= (self.width / 2):
             raise Exception("radius of vehicle shapes should not be less than from half of the vehicle width")

    def car_shape(self):
        start_point_of_circles = (self.radius ** 2 - (self.width / 2) ** 2) ** (1 / 2)
        circle_axis = self.length - (2 * start_point_of_circles)
        x = circle_axis / (self.circles_count - 1)
        pos = []
        for i in range(self.circles_count):
            pos.append([(x * i) - (self.length / 2 - start_point_of_circles), 0])
        self.pos = np.array(pos)

    def alignment(self, midpoints):
        orientation = psi(midpoints)
        turned_points = np.empty(shape=(len(orientation), self.circles_count, 2))
        for i in range(0, len(midpoints)):
            turned_points[i] = self.rotate(midpoints[i], self.pos, orientation[i])
        return turned_points

    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        origin = np.tile(origin, (self.circles_count,1))
        rot_mat = np.array([
                                [np.cos(angle), -1.*np.sin(angle)],
                                [np.sin(angle), np.cos(angle)    ]
                            ])
        rotated_points = origin + np.matmul(rot_mat, point.T).T
        return rotated_points

    def transform_point(self, midpoint, rotated_point):
        rotated_point[0] += midpoint[0]
        rotated_point[1] += midpoint[1]
        return rotated_point
