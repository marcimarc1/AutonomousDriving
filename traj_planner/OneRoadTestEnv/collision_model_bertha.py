# class for the vehicle shape and maneuver area
import numpy as np
import math


class Vehicle:
    def __init__(self, circles, length, width, r_veh):
        self.circles_count = circles
        self.length = length
        self.width = width
        self.pos = []
        self.radius = r_veh
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
        self.pos = pos

    def alignment(self, midpoints, orientation):

        turned_points = np.empty(shape=(len(orientation), self.circles_count, 2))
        for i in range(0, len(midpoints)):
            for j in range(0, len(self.pos)):
                rotated_point = self.rotate(midpoints[i], self.pos[j], orientation[i])#@arash if you transform coordinates, use the right coordinate frame
                                                                                     # i.e. look into "Engineering Dynamics"
                turned_points[i][j] = rotated_point#(self.transform_point(midpoints[i], rotated_point))
        return turned_points

    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point
        rotated_point = np.empty(shape=(2))
        rotated_point[0] = ox + math.cos(angle) * (px ) - math.sin(angle) * (py )
        rotated_point[1] = oy + math.sin(angle) * (px ) + math.cos(angle) * (py )
        return rotated_point

    def transform_point(self, midpoint, rotated_point):
        rotated_point[0] += midpoint[0]
        rotated_point[1] += midpoint[1]
        return rotated_point
