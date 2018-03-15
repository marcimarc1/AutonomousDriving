import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches

import fvks.scenario
import fvks.scenario.lanelet
import fvks.visualization.draw_dispatch
import pyfvks
from .util import draw_polygon_as_patch

from fvks.geometry.transform import to_homogeneous_coordinates
from fvks.geometry.transform import from_homogeneous_coordinates
import fvks.geometry.transform


def create_default_draw_params():
    draw_params = {'simulated_car': {'facecolor': '#3399ff',
                                     'edgecolor': '#0066cc',
                                     'draw_icon': False,
                                     'draw_bounding_box': True,
                                     'show_label': False,
                                     'trajectory_steps': 25,
                                     'zorder:': 20},
                   'trajectory': {'facecolor': '#b0b0b0'},
                   'lanelet': {'left_bound_color': '#555555',
                               'right_bound_color': '#555555',
                               'center_bound_color': '#dddddd',
                               'draw_left_bound': True,
                               'draw_right_bound': True,
                               'draw_center_bound': True,
                               'draw_border_vertices': False,
                               'draw_start_and_direction': True,
                               'show_label': False}}
    return {'scenario': draw_params}


def draw_scenario(obj, ax, draw_params, draw_func, call_stack):
    call_stack = tuple(list(call_stack) + ['scenario'])
    fvks.visualization.draw_dispatch.draw_object(
        obj.get_obstacles_by_role_and_type(), ax, draw_params, draw_func, 
        call_stack)
    fvks.visualization.draw_dispatch.draw_object(
        obj.lanelet_network, ax, draw_params, draw_func, call_stack)


def draw_obstacle_container(obj, ax, draw_params, draw_func, call_stack):
    fvks.visualization.draw_dispatch.draw_object(
        obj.stored_object, ax, draw_params, draw_func, call_stack)


def draw_trajectory(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'trajectory', 'facecolor'))
    except KeyError:
        print("Cannot find stylesheet for trajectory. Called through:")
        print(call_stack)
    """
    if "time_to_color" in draw_params.keys():
        raise Exception()  # TODO
        if len(v) > 0:
            for vv_idx in range(0, len(v) - 1):
                ax.plot(v[vv_idx:vv_idx + 2, 0], v[vv_idx:vv_idx + 2, 1], '-',
                        color=draw_params['time_to_color'].to_rgba(
                            vv_idx + obj.t0))
    else:
        if len(v) > 0:
            ax.plot(v[:, 0], v[:, 1], '-', color=draw_params['color'])
    """
    for s in obj.state_list:
        patch = patches.Ellipse(s.position, 0.25, 0.25, zorder=10)
        ax.add_patch(patch)


def draw_lanelet_network(obj, ax, draw_params, draw_func, call_stack):
    call_stack = tuple(list(call_stack) + ['lanelet_network'])
    for lanelet in obj.lanelets:
        fvks.visualization.draw_dispatch.draw_object(
            lanelet, ax, draw_params, draw_func, call_stack)


def draw_lanelet(obj, ax, draw_params, draw_func, call_stack):
    try:
        left_bound_color = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'left_bound_color'))
        right_bound_color = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'right_bound_color'))
        center_bound_color = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'center_bound_color'))
        show_label = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'show_label'))
        draw_border_vertices = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'draw_border_vertices'))
        draw_left_bound = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'draw_left_bound'))
        draw_right_bound = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'draw_right_bound'))
        draw_center_bound = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'draw_center_bound'))
        draw_start_and_direction = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'lanelet', 'draw_start_and_direction'))
    except KeyError:
        print("Cannot find stylesheet for lanelet. Called through:")
        print(call_stack)
    
    def direction(x, y, angle, scale, color='#dddddd'):
        pts = np.array([[0.0, -0.5, 1.0],
                        [1.0, 0.0, 1.0],
                        [0.0, 0.5, 1.0],
                        [0.0, -0.5, 1.0]])
        scale_m = np.array([[scale, 0, 0],
                            [0, scale, 0],
                            [0, 0, 1]])
        transform = np.array([[np.cos(angle), -np.sin(angle), x],
                              [np.sin(angle), np.cos(angle), y],
                              [0, 0, 1]])
        ptr_trans = transform.dot(scale_m.dot(pts.transpose()))
        ptr_trans = ptr_trans[0:2, :]
        ptr_trans = ptr_trans.transpose()
        codes = [Path.MOVETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.CLOSEPOLY,
                 ]
        path = Path(ptr_trans, codes)
        patch = patches.PathPatch(path, color=color,
                                  lw=0.5, zorder=10.1)
        ax.add_patch(patch)

    if draw_right_bound:
        ax.plot(obj.right_vertices[:, 0], obj.right_vertices[:, 1], '-',
                color=right_bound_color,
                lw=0.5,
                zorder=10)
    if draw_left_bound:
        ax.plot(obj.left_vertices[:, 0], obj.left_vertices[:, 1], '-',
                color=left_bound_color,
                lw=0.5,
                zorder=10)
    if draw_center_bound:
        ax.plot(obj.center_vertices[:, 0],
                obj.center_vertices[:, 1],
                '-', color=center_bound_color,
                zorder=10)

    if draw_start_and_direction:
        ax.plot([obj.left_vertices[0, 0], obj.right_vertices[0, 0]],
                [obj.left_vertices[0, 1], obj.right_vertices[0, 1]],
                '-',
                color=center_bound_color,
                zorder=10)
        
        center = 0.5 * (obj.left_vertices[0] + obj.right_vertices[0])
        tan_vec = obj.right_vertices[0] - obj.left_vertices[0]
        direction(center[0], center[1],
                np.arctan2(tan_vec[1], tan_vec[0]) + 0.5 * np.pi, 1.5,
                color=center_bound_color)

    if show_label:
        text_pos = obj.pos_interpolate(0.5 * obj.distance[-1])[0]
        ax.text(text_pos[0], text_pos[1],
                str(obj.lanelet_id),
                bbox={'facecolor': center_bound_color, 'pad': 2},
                horizontalalignment='center', verticalalignment='center',
                zorder=10.2)

    if draw_border_vertices:
        for p in obj.left_vertices:
            ax.add_patch(patches.Ellipse(p, 0.3, 0.3, color=left_bound_color))
        for p in obj.right_vertices:
            ax.add_patch(patches.Ellipse(p, 0.3, 0.3, color=right_bound_color))


def draw_simulated_car(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'simulated_car', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'simulated_car', 'edgecolor'))
        trajectory_steps = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'simulated_car', 'trajectory_steps'))
        draw_icon = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'simulated_car', 'draw_icon'))
        show_label = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'simulated_car', 'show_label'))
        draw_bounding_box = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('scenario', 'simulated_car', 'draw_bounding_box'))
    except KeyError:
        print("Cannot find stylesheet for simulated_car. Called through:")
        print(call_stack)

    if obj.active:
        vertices = [(- 0.5 * obj.length,
                     - 0.5 * obj.width),
                    (+ 0.5 * obj.length,
                     - 0.5 * obj.width),
                    (+ 0.5 * obj.length,
                     + 0.5 * obj.width),
                    (- 0.5 * obj.length,
                     + 0.5 * obj.width)]
        h_vertices = to_homogeneous_coordinates(np.array(vertices))
        t_m_a = fvks.geometry.transform.translation_rotation_matrix(
            np.array([0, 0]), obj.orientation)
        t_m_b = fvks.geometry.transform.translation_rotation_matrix(
            obj.position, 0.0)
        t_m = t_m_b.dot(t_m_a)
        t_vertices = from_homogeneous_coordinates(
          t_m.dot(h_vertices.transpose()).transpose())

        if draw_bounding_box:
            draw_polygon_as_patch(t_vertices, ax, zorder=25,
                                  facecolor=facecolor,
                                  edgecolor=edgecolor, lw=0.5)
        for time_idx in range(obj.current_time_idx, obj.current_time_idx +
            trajectory_steps):
            tmp = obj.trajectory.get_state_at_time(time_idx)
            if tmp is not None:
                patch = patches.Ellipse(tmp.position, 0.25, 0.25, zorder=24)
                ax.add_patch(patch)
        if draw_icon:
            draw_car(obj.position[0], obj.position[1], obj.orientation, 2.5,
                     ax, zorder=30)
        if show_label:
            ax.text(obj.position[0]+0.5, obj.position[1], str(obj.obstacle_id),
                    clip_on=True, zorder=1000)


def draw_car(pos_x, pos_y, rotate, scale, ax, zorder=5, carcolor='#ffffff',
             lw=0.5):
    rotate = rotate + np.pi

    def draw_filled(verts, ax):
        codes = [Path.MOVETO]
        for p in verts:
            codes.append(Path.LINETO)
        codes.append(Path.CLOSEPOLY)
        patch = patches.PathPatch(path, facecolor='orange', lw=2)
        ax.plot(verts[:, 0], verts[:, 1])
        ax.add_patch(patch)

    def reshape_and_addup(verts):
        verts = verts.reshape((int(len(verts)/2), 2))
        for i in range(1, len(verts)):
            verts[i] = verts[i]+verts[i-1]
        return verts

    def transform(vertices, pos_x, pos_y, rotate, scale):
        vertices = np.asarray(np.bmat([vertices, np.ones((len(vertices), 1))]))
        tmat = np.array([[np.cos(rotate), -np.sin(rotate), pos_x],
                         [np.sin(rotate), np.cos(rotate), pos_y], [0, 0, 1]])
        scalemat = np.array([[scale*1/356.3211, 0, 0], [0, scale*1/356.3211, 0],
                             [0, 0, 1]])
        centermat = np.array([[1, 0, -250], [0, 1, -889], [0, 0, 1]])
        tmat = tmat.dot(scalemat.dot(centermat))
        vertices = tmat.dot(vertices.transpose())
        vertices = np.array([[1, 0, 0], [0, 1, 0]]).dot(vertices).transpose()
        return vertices

    verts1 = np.array([193.99383, 752.94359,
                       -22.66602, 38,
                       -9.33398, 52.66797,
                       -2.60743, 44.82812,
                       -0.0586, 0,
                       0.0293, 0.91992,
                       -0.0293, 0.91797,
                       0.0586, 0,
                       2.60743, 44.82813,
                       9.33398, 52.66797,
                       22.66602, 38.00003,
                       59.33398, -7.334,
                       62, -10.666,
                       -6, -50.00003,
                       -2.65234, -68.41407,
                       2.65234, -68.41601,
                       6, -50,
                       -62, -10.66602])

    verts2 = np.array([715.99381, 768.27757,
                       -101.332, 6.66602,
                       10.666, 62.66797,
                       3.3223, 50.41797,
                       -3.3223, 50.41796,
                       -10.666, 62.66601,
                       101.332, 6.668,
                       22, -42.66799,
                       9.9824, -76.83594,
                       # 0.018,0,
                       # -0.01,-0.24804,
                       # 0.01,-0.24805,
                       # -0.018,0,
                       -9.9824, -76.83789,
                       0, 0])

    verts3 = np.array([421.06111, 751.61113,
                       190.2667, 3.33333,
                       13.3333, 5.33334,
                       -108.6666, 12.66666,
                       -134, 0,
                       -119.3334, -18.66666,
                       127.6456, -2.96473])

    verts4 = np.array([271.32781, 712.14446,
                       -6, -0.8,
                       -16, 12,
                       -14.8, 19.2,
                       -4, 6,
                       20.4, 0.4,
                       3.6, -4.4,
                       4.8, -2.8])
    verts5 = np.array([191.32781, 753.94359,
                       -99.999996, 11,
                       -63, 18.5,
                       -59, 38.5,
                       -20, 77,
                       20, 59.52734,
                       57, 36.49998,
                       65, 20.49999,
                       99.999996, 11.0001])

    verts6 = np.array([421.06111, 1027.399,
                       190.2667, -3.3333,
                       13.3333, -5.3333,
                       -108.6666, -12.6667,
                       -134, 0,
                       -119.3334, 18.6667,
                       127.6456, 2.9647])

    verts7 = np.array([271.32781, 1066.8657,
                       -6, 0.8,
                       -16, -12,
                       -14.8, -19.2,
                       -4, -6,
                       20.4, -0.4,
                       3.6, 4.4,
                       4.8, 2.8])

    verts8 = np.array([389.79851, 728.34788,
                       -343.652396, 10.16016,
                       -68.666, 22.42969,
                       -29.2558, 74.57031,
                       -7.3164, 60.35742,
                       -0.074, 0,
                       0.037, 0.76758,
                       -0.037, 0.76758,
                       0.074, 0,
                       7.3164, 42.35937,
                       29.2558, 74.57031,
                       68.666, 22.4278,
                       343.652396, 10.1621,
                       259.5859, -4.6192,
                       130.2539, -17.5527,
                       24.0196, -18.4766,
                       17.5527, -65.58788,
                       3.6953, -37.42773,
                       0, -13.24414,
                       -3.6953, -55.42774,
                       -17.5527, -65.58984,
                       -24.0196, -18.47656,
                       -130.2539, -17.55274])

    verts1 = reshape_and_addup(verts1)  # vorderscheibe
    verts2 = reshape_and_addup(verts2)  # rueckscheibe
    verts3 = reshape_and_addup(verts3)  # seitenscheibe rechts
    verts4 = reshape_and_addup(verts4)  # rueckspiegel links
    verts5 = reshape_and_addup(verts5)  # motorhaube
    verts6 = reshape_and_addup(verts6)  # fenster rechts
    verts7 = reshape_and_addup(verts7)  # rueckspiegel rechts
    verts8 = reshape_and_addup(verts8)  # umriss

    verts1 = transform(verts1, pos_x, pos_y, rotate, scale)
    verts2 = transform(verts2, pos_x, pos_y, rotate, scale)
    verts3 = transform(verts3, pos_x, pos_y, rotate, scale)
    verts4 = transform(verts4, pos_x, pos_y, rotate, scale)
    verts5 = transform(verts5, pos_x, pos_y, rotate, scale)
    verts6 = transform(verts6, pos_x, pos_y, rotate, scale)
    verts7 = transform(verts7, pos_x, pos_y, rotate, scale)
    verts8 = transform(verts8, pos_x, pos_y, rotate, scale)

    windowcolor = '#555555'
    draw_polygon_as_patch(verts1, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts2, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts3, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts4, ax, facecolor='#ffffff', zorder=zorder + 1,
                          lw=lw)
    ax.plot(verts5[:, 0], verts5[:, 1], zorder=zorder+1, color='#000000',
            lw=lw)
    draw_polygon_as_patch(verts6, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts7, ax, facecolor='#ffffff', zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts8, ax, facecolor=carcolor, edgecolor='#000000',
                          zorder=zorder, lw=lw)


draw_func_dict = {fvks.scenario.scenario.Scenario: draw_scenario,
                  fvks.scenario.lanelet.Lanelet: draw_lanelet,
                  fvks.scenario.lanelet.LaneletNetwork: draw_lanelet_network,
                  fvks.scenario.traffic.SimulatedCar: draw_simulated_car,
                  fvks.scenario.trajectory.Trajectory: draw_trajectory,
                  fvks.scenario.obstacle.ObstacleContainer:
                      draw_obstacle_container}
