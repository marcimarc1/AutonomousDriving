import numpy as np
import fvks.geometry.bvh
import copy
from .scenario_exception import ScenarioError
import fvks.geometry.transform


class LaneletNetwork:

    @classmethod
    def create_from_lanelet_list(cls, lanelets):
        lanelet_network = cls()
        for l in lanelets:
            lanelet_network.add_lanelet(copy.deepcopy(l))
        return lanelet_network

    @classmethod
    def create_from_lanelet_network(cls, lanelet_network):
        new_lanelet_network = cls()
        for l in lanelet_network.lanelets:
            new_lanelet_network.add_lanelet(copy.deepcopy(l))
        return new_lanelet_network

    def __init__(self):
        self.lanelets = []
        self.lanelet_bvh = None

    def find_lanelet_by_id(self, lanelet_id):
        for l in self.lanelets:
            if l.lanelet_id == lanelet_id:
                return l
        raise ScenarioError

    def add_lanelet(self, lanelet):
        if type(lanelet) == list:
            for l in lanelet:
                self.add_lanelet(l)
        else:
            try:
                self.find_lanelet_by_id(lanelet.lanelet_id)
            except ScenarioError:
                self.lanelets.append(lanelet)
                self.lanelet_bvh = None
            else:
                raise ScenarioError

    def add_lanelet_network(self, lanelet_network):
        for l in lanelet_network.lanelets:
            self.add_lanelet(l)

    def translate_rotate(self, translation, angle):
        for lanelet in self.lanelets:
            lanelet.translate_rotate(translation, angle)
        self.lanelet_bvh = None


    def find_nearest_point_on_lanelet(self, pt, dist_threshold=10):
        """

        :param pt:
        :param dist_threshold:
        :return: closest point on lanelet,
                 distance along lanelet to close point
                 closest lanelet
                 segment index of closest point within lanelet
                 list of all lanelet segments, which were examined
        """
        if not self.lanelets:
            return None
        if not self.lanelet_bvh:
            print("Build lanelet BVH")
            bv_list = list()
            for lanelet in self.lanelets:
                for s_idx in range(0, len(lanelet.center_vertices) - 1):
                    pt1 = lanelet.center_vertices[s_idx]
                    pt2 = lanelet.center_vertices[s_idx + 1]
                    bv_e = fvks.geometry.bvh.BvhElement()
                    bv_e.stored_objects = [LaneletSegmentBV(lanelet, pt1, pt2,
                                                            s_idx)]
                    bv_e.bounding_volume = [min(pt1[0], pt2[0]),
                                            max(pt1[0], pt2[0]),
                                            min(pt1[1], pt2[1]),
                                            max(pt1[1], pt2[1])]
                    bv_e.grow_empty_bounding_volume()
                    bv_list.append(bv_e)

            root_node = fvks.geometry.bvh.BvhElement()
            root_node.depth = 0
            root_node.is_container = False
            root_node.add_children(bv_list)
            root_node.partition(10)

            self.lanelet_bvh = root_node

        result = self.lanelet_bvh.query([pt[0] - dist_threshold,
                                         pt[0] + dist_threshold,
                                         pt[1] - dist_threshold,
                                         pt[1] + dist_threshold])[0]
        if not result:
            print("Point not in BVH of lanelets")
            return None

        lanelet_segment_min = result[0]
        d_min = fvks.geometry.polyline.line_segment_nearest_point(
            lanelet_segment_min.pt1, lanelet_segment_min.pt2, pt)[1]
        for r in result:
            d = fvks.geometry.polyline.line_segment_nearest_point(r.pt1, r.pt2,
                                                                  pt)[1]
            if d < d_min:
                lanelet_segment_min = r
                d_min = d

        pt_proj, dist, rd = fvks.geometry.polyline.line_segment_nearest_point(
            lanelet_segment_min.pt1, lanelet_segment_min.pt2, pt)
        return (pt_proj,
                lanelet_segment_min.lanelet.distance[lanelet_segment_min.segment_idx] + rd,
                lanelet_segment_min.lanelet,
                lanelet_segment_min.segment_idx,
                result)

    def __str__(self):
        return_str = ''
        for l in self.lanelets:
            return_str += '{:8d} lanelet\n'.format(l.lanelet_id)
        return return_str


class Lanelet():

    def __init__(self, left_vertices, center_vertices, right_vertices,
                 lanelet_id,
                 predecessor=None, successor=None,
                 adjacent_left=None, adjacent_left_same_direction=None,
                 adjacent_right=None, adjacent_right_same_direction=None,
                 speed_limit=None):
        if (len(left_vertices) != len(center_vertices) and
                    len(center_vertices) != len(right_vertices)):
            raise ScenarioError
        self.left_vertices = left_vertices
        self.center_vertices = center_vertices
        self.right_vertices = right_vertices
        if predecessor is None:
            self.predecessor = []
        else:
            self.predecessor = predecessor
        if successor is None:
            self.successor = []
        else:
            self.successor = successor
        self.adj_left = adjacent_left
        self.adj_left_same_direction = adjacent_left_same_direction
        self.adj_right = adjacent_right
        self.adj_right_same_direction = adjacent_right_same_direction

        self.lanelet_id = lanelet_id
        self.speed_limit = speed_limit
        self.distance = [0]
        for i in range(1, len(self.center_vertices)):
            self.distance.append(self.distance[i-1] +
                                 np.linalg.norm(self.center_vertices[i] -
                                                self.center_vertices[i-1]))
        self.distance = np.array(self.distance)

    def pos_interpolate(self, distance):
        if distance > self.distance[-1] or distance < 0:
            return None
        idx = 0
        while not (self.distance[idx] <= distance <= self.distance[idx+1]):
            idx += 1
        r = (distance - self.distance[idx])/(self.distance[idx+1] -
                                             self.distance[idx])
        return ((1 - r) * self.center_vertices[idx] +
                r * self.center_vertices[idx + 1],
                (1 - r) * self.right_vertices[idx] +
                r * self.right_vertices[idx + 1],
                (1 - r) * self.left_vertices[idx] +
                r * self.left_vertices[idx + 1],
                idx)

    def translate_rotate(self, translation, angle):
        t_m = fvks.geometry.transform.translation_rotation_matrix(translation,
                                                                  angle)

        tmp = t_m.dot(np.vstack((self.center_vertices.transpose(),
                                 np.ones((1, self.center_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self.center_vertices = tmp.transpose()

        tmp = t_m.dot(np.vstack((self.left_vertices.transpose(),
                                 np.ones((1, self.left_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self.left_vertices = tmp.transpose()

        tmp = t_m.dot(np.vstack((self.right_vertices.transpose(),
                      np.ones((1, self.right_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self.right_vertices = tmp.transpose()

    def cut(self, distance_begin, distance_end):
        float_tolerance = 1e-6
        if not (0 <= distance_begin < distance_end <= self.distance[-1]):
            return None
        begin = self.pos_interpolate(distance_begin)
        end = self.pos_interpolate(distance_end)
        idx_add_begin = begin[3]+1
        idx_add_end = end[3]
        left_vertices = np.vstack((np.array([begin[2]]),
                                   self.left_vertices[idx_add_begin:idx_add_end+1],
                                   np.array([end[2]])))
        center_vertices = np.vstack((np.array([begin[0]]),
                                     self.center_vertices[idx_add_begin:idx_add_end + 1],
                                     np.array([end[0]])))
        right_vertices = np.vstack((np.array([begin[1]]),
                                    self.right_vertices[idx_add_begin:idx_add_end + 1],
                                    np.array([end[1]])))
        return Lanelet(center_vertices=center_vertices,
                       left_vertices=left_vertices,
                       right_vertices=right_vertices,
                       predecessor=[],
                       successor=[],
                       adjacent_left=None,
                       adjacent_left_same_direction=None,
                       adjacent_right=None,
                       adjacent_right_same_direction=None,
                       lanelet_id=-1,
                       speed_limit=None)

    def concatenate(self, lanelet):
        #
        # NOT TESTED
        #
        float_tolerance = 1e-6
        if (np.linalg.norm(self.center_vertices[-1] - lanelet.center_vertices[0]) > float_tolerance or
            np.linalg.norm(self.left_vertices[-1] - lanelet.left_vertices[0]) > float_tolerance or
            np.linalg.norm(self.right_vertices[-1] - lanelet.right_vertices[0]) > float_tolerance):
            return None
        left_vertices = np.vstack((self.left_vertices,
                                   lanelet.left_vertices[1:]))
        center_vertices = np.vstack((self.center_vertices,
                                     lanelet.center_vertices[1:]))
        right_vertices = np.vstack((self.right_vertices,
                                    lanelet.right_vertices[1:]))
        return Lanelet(center_vertices=center_vertices,
                       left_vertices=left_vertices,
                       right_vertices=right_vertices,
                       predecessor=self.predecessor.copy(),
                       successor=lanelet.successor.copy(),
                       adjacent_left=None,
                       adjacent_left_same_direction=None,
                       adjacent_right=None,
                       adjacent_right_same_direction=None,
                       lanelet_id=-1,
                       speed_limit=None)

    def __str__(self):
        return 'Lane id:' + str(self.id)


class LaneletSegmentBV:
    def __init__(self, lanelet, pt1, pt2, segment_idx):
        self.pt1 = pt1
        self.pt2 = pt2
        self.lanelet = lanelet
        self.segment_idx = segment_idx
