class BvhElement():
    def __init__(self):
        self.is_container = True
        # Invariant: either children_nodes or stored_object list is empty
        self.children_nodes = list()
        self.stored_objects = list()
        self.bounding_volume = None
        self.depth = 0

    def add_children(self, object_list):
        assert (not self.is_container)
        self.bounding_volume = self.merge_bounding_volumes(
            self.bounding_volume, self.calc_bounding_volume(object_list))
        self.children_nodes += object_list
        self.grow_empty_bounding_volume()

    def grow_empty_bounding_volume(self):
        """
        Increase the bounding_volume if it zero otherwise
        :return:
        """
        EPSILON_LENGTH = 1e-6
        if (self.bounding_volume[1]-self.bounding_volume[0]) < EPSILON_LENGTH:
            self.bounding_volume[0] -= 0.5 * EPSILON_LENGTH
            self.bounding_volume[1] += 0.5 * EPSILON_LENGTH

        if (self.bounding_volume[3]-self.bounding_volume[2]) < EPSILON_LENGTH:
            self.bounding_volume[2] -= 0.5 * EPSILON_LENGTH
            self.bounding_volume[3] += 0.5 * EPSILON_LENGTH


    def calc_bounding_volume(self, object_list):
        """
        Calculate the bounding volume of a list of BvhElement

        :param object_list: list of BvhElement
        :return: list with [x_min x_max y_min y_max]
        """

        if not object_list:
            return None
        bounding_volume = object_list[0].bounding_volume.copy()
        for o in object_list:
            bounding_volume = self.merge_bounding_volumes(bounding_volume,
                                                          o.bounding_volume)


        return bounding_volume

    def merge_bounding_volumes(self, vol_a, vol_b):
        """
        Calculate the bounding volume of to bounding volumes

        :param vol_a:
        :param vol_b:
        :return:
        """
        if not vol_a:
            return vol_b
        elif not vol_b:
            return vol_a
        else:
            return [min(vol_a[0], vol_b[0]),
                    max(vol_a[1], vol_b[1]),
                    min(vol_a[2], vol_b[2]),
                    max(vol_a[3], vol_b[3])]

    def overlap(self, vol_a, vol_b):
        def overlap_interval(int_a, int_b):
            return max(0, (int_a[1] - int_a[0]) + (int_b[1] - int_b[0]) -
                       (max(int_a[1], int_b[1]) - min(int_a[0], int_b[0])))

        return (overlap_interval(vol_a[0:2], vol_b[0:2]) *
                overlap_interval(vol_a[2:4], vol_b[2:4]))

    def partition(self, param):
        assert (param >= 2)
        if self.is_container or len(self.children_nodes) <= param:
            return
        if (self.bounding_volume[1] - self.bounding_volume[0] >=
                    self.bounding_volume[3] - self.bounding_volume[2]):
            child_a_bv = [self.bounding_volume[0], 0.5 * (self.bounding_volume[0] + self.bounding_volume[1]),
                          self.bounding_volume[2], self.bounding_volume[3]]
            child_b_bv = [0.5 * (self.bounding_volume[0] + self.bounding_volume[1]), self.bounding_volume[1],
                          self.bounding_volume[2], self.bounding_volume[3]]
        else:
            child_a_bv = [self.bounding_volume[0], self.bounding_volume[1],
                          self.bounding_volume[2], 0.5 * (self.bounding_volume[2] + self.bounding_volume[3])]
            child_b_bv = [self.bounding_volume[0], self.bounding_volume[1],
                          0.5 * (self.bounding_volume[2] + self.bounding_volume[3]), self.bounding_volume[3]]
        child_a_obj = list()
        child_b_obj = list()
        for o in self.children_nodes:
            if (self.overlap(o.bounding_volume, child_a_bv) >=
                    self.overlap(o.bounding_volume, child_b_bv)):
                child_a_obj.append(o)
            else:
                child_b_obj.append(o)
        if (not child_a_obj) or (not child_b_obj):
            # in this case maybe a split along the other axis should be tested?
            return

        child_a = BvhElement()
        child_a.is_container = False
        child_a.add_children(child_a_obj)
        child_a.depth = self.depth + 1

        child_b = BvhElement()
        child_b.is_container = False
        child_b.add_children(child_b_obj)
        child_b.depth = self.depth + 1

        self.children_nodes = [child_a, child_b]
        child_a.partition(param)
        child_b.partition(param)

    def query(self, aa_window):
        result = list()
        num = 1
        if self.overlap(aa_window, self.bounding_volume) > 0:
            if self.is_container:
                result += self.stored_objects
            else:
                for c in self.children_nodes:
                    # result += c.query(aa_window)
                    tmp = c.query(aa_window)
                    result += tmp[0]
                    num += tmp[1]
        return result, num
