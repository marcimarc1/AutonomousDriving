import xml.etree.ElementTree as et
import numpy as np


class Pointlist:
    def __init__(self, points):
        self.points = points

    @classmethod
    def create_from_xml_node(cls, xml_node):
        point_list = []
        for point_node in xml_node.findall("point"):
            point_list.append(Point.create_from_xml_node(point_node))
        return cls(point_list)

    @classmethod
    def create_from_numpy_array(cls, points):
        point_list = []
        for point in points:
            point_list.append(Point.create_from_numpy_array(point))
        return cls(point_list)

    def as_numpy_array(self):
        tmp = []
        for point in self.points:
            tmp.append([point.x, point.y])
        return np.array(tmp)

    def add_points_to_node(self, xml_node):
        for point in self.points:
            xml_node.append(point.create_node())


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def as_numpy_array(self):
        return np.array([self.x, self.y])

    @classmethod
    def create_from_numpy_array(cls, point):
        return cls(point[0], point[1])

    @classmethod
    def create_from_xml_node(cls, xml_node):
        x = float(xml_node.find('x').text)
        y = float(xml_node.find('y').text)
        return cls(x, y)

    def create_node(self):
        point_node = et.Element('point')
        x = et.Element('x')
        x.text = str(np.float64(self.x))
        point_node.append(x)
        y = et.Element('y')
        y.text = str(np.float64(self.y))
        point_node.append(y)
        return point_node


class ExactValue:
    def __init__(self, value):
        self.value = value

    def create_node(self):
        node = et.Element('exact')
        node.text = str(np.float64(self.value))
        return node


class UncertainIntervalScalar:
    def __init__(self, value_lo, value_hi):
        self.value_lo = value_lo
        self.value_hi = value_hi

    def create_node(self):
        node_lo = et.Element('intervalStart')
        node_lo.text = str(np.float64(self.value_lo))
        node_hi = et.Element('intervalEnd')
        node_hi.text = str(np.float64(self.value_hi))
        return [node_lo, node_hi]

