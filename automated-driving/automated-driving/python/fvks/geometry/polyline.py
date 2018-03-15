import numpy as np
import fvks.geometry.bvh
import fvks.geometry.util


def ccw(pt_a, pt_b, pt_c):
    print("ccw in fvks.geometry.polyline is deprecated; use instead ccw in")
    print("fvks.geometry.util.")
    return fvks.geometry.util.ccw(pt_a, pt_b, pt_c)

def signed_angle_difference(vec_a, vec_b):
    """
     pi/2 = signed_angle_difference(np.array([1, 0]), np.array([0, 1]))
    -pi/2 = signed_angle_difference(np.array([0, 1]), np.array([1, 0]))
    """
    phi = (np.arccos(vec_a.dot(vec_b) / 
           (np.linalg.norm(vec_a) * np.linalg.norm(vec_a))))
    origin_vec = np.array([0, 0])
    if ccw(origin_vec, vec_a, vec_b) < 0:
        return -phi
    else:
        return phi

def calculate_orientation_from_polyline(polyline):
    if (len(polyline)<2):
        raise NameError('Cannot create orientation from polyline of length < 2')

    orientation = []
    for i in range(0, len(polyline)-1):
        pt1 = polyline[i]
        pt2 = polyline[i+1]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    for i in range(len(polyline)-1, len(polyline)):
        pt1 = polyline[i-1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    return np.array(orientation)


def polyline_nearest_point(polyline, pt):
    if len(polyline)<2:
        return None
    pt_nearest, d, rd = line_segment_nearest_point(polyline[0], polyline[1], pt)
    segment_idx = 0
    for i in range(1,len(polyline)-1):
        pt_nearest_i, d_i, rd_i = line_segment_nearest_point(
            polyline[i], polyline[i + 1], pt)
        if d_i<d:
            d = d_i
            pt_nearest = pt_nearest_i
            rd = rd_i
            segment_idx = i
    return (pt_nearest, d, rd, segment_idx)

def line_segment_nearest_point(line_pt_a, line_pt_b, pt):
    """
    closest point on the line segment
    :param line_pt_a:
    :param line_pt_b:
    :param pt:
    :return:  pt_proj: closest point on line segment
        dist:    distance pt to closest point on line segment
        rd:      distance from line_pt_a along line segment to closest point
    """

    # TODO: make sure that line_pt_a, line_pt_b and pt are 2x1 matrices

    d = line_pt_b-line_pt_a
    v = pt-line_pt_a

    c = (d.transpose().dot(v))/(d.transpose().dot(d))*d

    rd = (d.transpose().dot(v))/np.linalg.norm(d)
    if rd<0:
        #rd = 0
        pt_proj = line_pt_a
        dist = np.linalg.norm(line_pt_a-pt)
    elif rd>np.linalg.norm(d):
        #rd = np.linalg.norm(d)
        pt_proj = line_pt_b
        dist = np.linalg.norm(line_pt_b-pt)
    else:
        dist = np.linalg.norm(v-c)
        pt_proj = line_pt_a+c

    # reshape pt_proj to 1x2
    return ( pt_proj.transpose(), dist, rd )


def frenet_coordinates(pt, bvh=None, polyline=None):
    assert (bvh or polyline)
    assert not bvh # TODO: add this
    pt_nearest, d, rd, segment_idx = polyline_nearest_point(polyline, pt)

def resample_polyline(polyline, step=2.0):
    new_polyline = [polyline[0]]
    current_position = 0 + step
    current_length = np.linalg.norm(polyline[0] - polyline[1])
    current_idx = 0
    while current_idx < len(polyline) - 1:
        if current_position >= current_length:
            current_position = current_position - current_length
            current_idx += 1
            if current_idx > len(polyline) - 2:
                break
            current_length = np.linalg.norm(polyline[current_idx + 1] 
                                            - polyline[current_idx])
        else:
            rel = current_position/current_length
            new_polyline.append((1-rel) * polyline[current_idx] +
                                rel * polyline[current_idx + 1])
            current_position += step
    return np.array(new_polyline)

def calculate_length_of_polyline(polyline, segment_end_idx, segment_start_idx=0):
    length = 0
    for i in range(segment_start_idx, segment_end_idx):
        length += np.linalg.norm(polyline[i]-polyline[i + 1])
    return length

def douglas_peucker(polyline, epsilon):
    # find point with maximum distance
    d_max = 0
    index = 0
    results = list()
    for i in range(1, len(polyline)-1):
        d = line_segment_nearest_point(polyline[0], polyline[-1], polyline[i])[1]
        if d > d_max:
            index = i
            d_max = d
    # simplify polyline if maximum distance is bigger than epsilon
    if d_max >= epsilon:
        results_1 = douglas_peucker(polyline[0:index], epsilon)
        results_2 = douglas_peucker(polyline[index:-1], epsilon)
        # store results
        results.extend(results_1)
        results.extend(results_2)
    else:
        results.append(polyline[0])
        results.append(polyline[-1])
    return results

