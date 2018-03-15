import numpy as np


def ccw(pt_a, pt_b, pt_c):
    return ((pt_b[0] - pt_a[0]) * (pt_c[1] - pt_a[1]) -
            (pt_b[1] - pt_a[1]) * (pt_c[0] - pt_a[0]))

def distance_point_line(pt, l_pt_1, l_pt_2):
    return (np.linalg.norm(np.cross(l_pt_2 - l_pt_1, l_pt_1 - pt)) / 
            np.linalg.norm(l_pt_2 - l_pt_1))

def intersection_line_line(l1_pt1, l1_pt2, l2_pt1, l2_pt2):
    """
    Intersection between two lines, defined by four points.
    """
    # http://www.ahristov.com/tutorial/geometry-games/intersection-lines.html
    x1 = l1_pt1[0]
    y1 = l1_pt1[1]
    x2 = l1_pt2[0]
    y2 = l1_pt2[1]
    x3 = l2_pt1[0]
    y3 = l2_pt1[1]
    x4 = l2_pt2[0]
    y4 = l2_pt2[1]
    
    d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    
    if np.abs(d) <= 0.000001:
        raise Exception()

    xi = ((x3-x4)*(x1*y2-y1*x2)-(x1-x2)*(x3*y4-y3*x4))/d
    yi = ((y3-y4)*(x1*y2-y1*x2)-(y1-y2)*(x3*y4-y3*x4))/d

    return np.array([xi, yi])