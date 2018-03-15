import numpy as np
import math
import sys


class Distance:
    def pseudo_distance(self, point0, point1, point2, point3, veh_position):
        """
        Computes the Pseudo compute_distance between two states.
        :param point0: one point before the point1 for tangents computation: it is a vector so it is array of x and y
        :param point1: first point of polygon: it is a point so it is array of x and y
        :param point2: second point of polygon: it is a point so it is array of x and y
        :param point3: one point after the point2 for tangents computation: it is a vector so it is array of x and y
        :param veh_position: vehicle position: it is a vector so it is array of x and y
        :return: compute_distance value
        """
        tangents_vector1, tangents_vector2 = self.pseudo_distance_tangnet(point0, point1, point2, point3)
        a = (tangents_vector2[0] - tangents_vector1[0]) * (point1[0] - point2[0]) + \
            (tangents_vector2[1] - tangents_vector1[1]) * (point1[1] - point2[1])
        b = (veh_position[0] - point1[0]) * (tangents_vector2[0] - tangents_vector1[0]) + tangents_vector1[0] * (
            point1[0] - point2[0]) + \
            (veh_position[1] - point1[1]) * (tangents_vector2[1] - tangents_vector1[1]) + tangents_vector1[1] * (
            point1[1] - point2[1])
        c = (veh_position[0] - point1[0]) * tangents_vector1[0] + (veh_position[1] - point1[1]) * tangents_vector1[1]

        if a != 0:
            if b ** 2 - 4 * a * c < 0:
                return self.find_norm_distance(point0, point1, point2, point3, veh_position)
            lambda1 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            lambda2 = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            lambda_ = lambda2
            if lambda1 > 0:
                lambda_ = lambda1
        elif b == 0:
            return np.linalg.norm((point0 - veh_position))
        else:
            lambda_ = -c / b

        distance = [veh_position[0] - lambda_ * point2[0] - (1 - lambda_) * point1[0],
                    veh_position[1] - lambda_ * point2[1] - (1 - lambda_) * point1[1]]
        self.distance_vector=distance
        norm_distance = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
        #TODO: based on paper there was -distance!!
        # if veh_position[1] <= 0:
        #     norm_distance = -norm_distance

        return norm_distance

    def pseudo_distance_tangnet(self, point0, point1, point2, point3):
        tangent1 = np.array(
            [(point1[0] - point0[0]),
             (point1[1] - point0[1])])

        tangent2 = np.array(
            [(point3[0] - point2[0]),
             (point3[1] - point2[1])])
        # tangent1 = tangent1 / tangent1[0]
        # tangent2 = tangent2 / tangent2[0]
        return tangent1, tangent2

    def find_norm_distance(self, point0, point1, point2, point3, veh_position):

        return min(self.compute_norm_distant(point0, point1, veh_position),
                   self.compute_norm_distant(point1, point2, veh_position),
                   self.compute_norm_distant(point2, point3, veh_position))

    def compute_norm_distant(self, point1, point2, veh_position):
        px = point2[0] - point1[0]
        py = point2[1] - point1[1]

        something = px * px + py * py

        u = ((veh_position[0] - point1[0]) * px + (veh_position[1] - point1[1]) * py) / float(something)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = point1[0] + u * px
        y = point1[1] + u * py

        dx = x - veh_position[0]
        dy = y - veh_position[1]

        dist = math.sqrt(dx * dx + dy * dy)

        return dist

    def euclidean_distance(self, x1, y1, x2, y2, x, y):
        """
        it is just for tests, use pseudo_distance_tangnet instead of that
        :param x1: x of first point
        :param y1: y of first point
        :param x2: x of second point
        :param y2: y of second point
        :param x: x of vehicle
        :param y: y of vehicle
        :return: compute_distance value
        """
        A = x - x1
        B = y - y1
        C = x2 - x1
        D = y2 - y1

        dot = A * C + B * D
        len_sq = C * C + D * D
        param = -1
        if len_sq != 0:
            param = dot / len_sq

        if param < 0:
            xx = x1
            yy = y1
        elif param > 1:
            xx = x2
            yy = y2
        else:
            xx = x1 + param * C
            yy = y1 + param * D

        dx = x - xx
        dy = y - yy
        return math.sqrt(dx * dx + dy * dy)

    def compute_distance_to_driver_corridors(self, veh_poses, left_boundary, right_boundary):
        ob_points = np.empty(shape=(4, 2))
        left_distance = np.empty(shape=(len(veh_poses)))
        right_distance = np.empty(shape=(len(veh_poses)))
        self.distance_vector = np.empty(shape=(1, 2))
        self.distance_vectors = np.empty(shape=(len(veh_poses), 2))
        for time_step_counter in range(0, len(veh_poses)):
            veh_pos = veh_poses[time_step_counter]
            # one vehicle point at each time step
            self.compute_distance_to_driver_corridor(left_boundary, left_distance, ob_points, time_step_counter,
                                                     veh_pos, veh_poses)
            self.compute_distance_to_driver_corridor(right_boundary, right_distance, ob_points, time_step_counter,
                                                     veh_pos, veh_poses)
        return left_distance, right_distance

    def compute_distance_to_driver_corridor(self, boundary_points, distance, ob_points, timestep_counter, veh_pos,
                                            veh_poses):

        distance[timestep_counter] = sys.maxsize
        for boundary_point_counter in range(0, len(boundary_points)):
            dist_temp = 0
            # if veh_pos[:, 0].shape > 0:
            #     number_of_iteration = len(veh_pos)
            # else:
            #     number_of_iteration = 1
            if hasattr(veh_pos[0], "__len__"):
                number_of_vehicle_points=len(veh_pos)
            else:
                number_of_vehicle_points=1
            for vehicle_circle_counter in range(0, number_of_vehicle_points):
                if hasattr(veh_pos[0], "__len__"):
                    veh_circle_pos = veh_pos[vehicle_circle_counter]
                else:
                    veh_circle_pos = veh_pos
                number_of_available_points = self.assign_boundary_points(boundary_point_counter, boundary_points,
                                                                         ob_points,
                                                                         veh_poses)
                if number_of_available_points == 3:
                    dis1 = self.euclidean_distance(ob_points[0][0], ob_points[0][1], ob_points[1][0], ob_points[1][1],
                                                   veh_circle_pos[0], veh_circle_pos[1])
                    dis2 = self.euclidean_distance(ob_points[1][0], ob_points[1][1], ob_points[2][0], ob_points[2][1],
                                                   veh_circle_pos[0], veh_circle_pos[1])
                    dist_temp += min(dis1, dis2)
                elif number_of_available_points == 2:
                    dist_temp += self.euclidean_distance(ob_points[0][0], ob_points[0][1], ob_points[1][0],
                                                         ob_points[1][1],
                                                         veh_circle_pos[0], veh_circle_pos[1])
                else:
                    dist_temp += self.pseudo_distance(ob_points[0], ob_points[1],
                                                      ob_points[2], ob_points[3], veh_circle_pos)
                    if dist_temp == "no answer":
                        print("ERROR in distance of :" + str(timestep_counter) + " and step " + str(
                            boundary_point_counter))
            dist_temp = dist_temp / number_of_vehicle_points
            if abs(dist_temp) < abs(distance[timestep_counter]):
                    distance[timestep_counter] = dist_temp
                    self.distance_vectors[timestep_counter] = self.distance_vector

    def assign_boundary_points(self, boundary_point_counter, boundary_points, ob_points, veh_poses):
        ob_points[0] = boundary_points[boundary_point_counter]
        number_of_available_points = 1
        if boundary_point_counter + 1 < len(boundary_points):
            ob_points[1] = boundary_points[boundary_point_counter + 1]
            number_of_available_points += 1
        if boundary_point_counter + 2 < len(boundary_points):
            ob_points[2] = boundary_points[boundary_point_counter + 2]
            number_of_available_points += 1
        if boundary_point_counter + 3 < len(boundary_points):
            ob_points[3] = boundary_points[boundary_point_counter + 3]
            number_of_available_points += 1
        return number_of_available_points

    def compute_distance(self, veh_poses, obstacles_points):
        distance = np.empty(shape=(len(obstacles_points), len(obstacles_points[0])))
        for number_of_obstacles in range(0, len(obstacles_points)):
            obstacle_points = obstacles_points[number_of_obstacles]
            for timestep_counter in range(0, len(obstacle_points)):
                veh_pos = veh_poses[timestep_counter]
                # one obstacle points at each timestep
                ob_points = obstacle_points[timestep_counter]
                dist_temp, distance[number_of_obstacles][timestep_counter] = sys.maxsize, sys.maxsize
                if len(ob_points) == 3:
                    dis1 = self.euclidean_distance(ob_points[0][0], ob_points[0][1], ob_points[1][0], ob_points[1][1],
                                                   veh_pos[0], veh_pos[1])
                    dis2 = self.euclidean_distance(ob_points[1][0], ob_points[1][1], ob_points[2][0], ob_points[2][1],
                                                   veh_pos[0], veh_pos[1])
                    dist_temp = min(dis1, dis2)
                elif len(ob_points) == 2:
                    dist_temp = self.euclidean_distance(ob_points[0][0], ob_points[0][1], ob_points[1][0],
                                                        ob_points[1][1],
                                                        veh_pos[0], veh_pos[1])

                for innerCounter in range(0, len(ob_points)):
                    if innerCounter + 3 >= len(ob_points):
                        break
                    dist_temp = self.pseudo_distance(ob_points[innerCounter], ob_points[innerCounter + 1],
                                                     ob_points[innerCounter + 2], ob_points[innerCounter + 3], veh_pos)
                    if (dist_temp == "no answer"):
                        print(timestep_counter)
                if dist_temp < distance[number_of_obstacles][timestep_counter]:
                    distance[number_of_obstacles][timestep_counter] = dist_temp
        return distance

    def get_n_lambda(self):
        return self.distance_vectors

"""
sample usage of this class
"""
# obstacleClass = Distance()
#
# # print(obstacleClass.compute_distance([[10, 0]], [[[[0, 2], [2, 2], [2, 0], [0, 0]]]]))
