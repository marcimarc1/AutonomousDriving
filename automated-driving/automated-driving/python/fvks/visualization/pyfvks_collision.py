import pyfvks
import fvks.visualization.draw_dispatch
import matplotlib.patches
import matplotlib.colors

from .util import draw_polygon_as_patch


def create_default_draw_params():
    draw_params = {'rectobb': {'facecolor': '#ff9999',
                               'edgecolor': '#000000',
                               'zorder': 20},
                   'rectaabb': {'facecolor': '#ff9999',
                                'edgecolor': '#000000',
                                'zorder': 20},
                   'triangle': {'facecolor': '#ff9999',
                                'edgecolor': '#000000',
                                'zorder': 20},
                   'circle': {'facecolor': '#ff9999',
                              'edgecolor': '#000000',
                              'zorder': 20},
                   'point': {'facecolor': '#ff9999',
                             'zorder': 20},
                   'boundingvolume': {'facecolor': 'none',
                                      'edgecolor': '#ff9999',
                                      'draw_recursively': True,
                                      'draw_contained_collision_object': True,
                                      'zorder': 20},
                   'polygon': {'facecolor': '#ff9999',
                               'edgecolor': '#000000',
                               'zorder': 20,
                               'draw_mesh': False}}
    return {'collision': draw_params}


def draw_collision_point(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'point', 'facecolor'),
            ('collision', 'facecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'point', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for point. Called through:")
        print(call_stack)
    ax.plot(obj.center()[0], obj.center()[1], color=facecolor, zorder=zorder)


def draw_collision_rectaabb(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for rectaabb. Called through:")
        print(call_stack)

    vertices = [(obj.min_x(), obj.min_y()),
                (obj.min_x(), obj.max_y()),
                (obj.max_x(), obj.max_y()),
                (obj.max_x(), obj.min_y())]

    draw_polygon_as_patch(vertices, ax, zorder=zorder, facecolor=facecolor,
                          edgecolor=edgecolor, lw=0.5)


def draw_collision_rectobb(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'rectobb', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'rectobb', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectobb', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for rectobb. Called through:")
        print(call_stack)
    center = obj.center()
    r_x = obj.r_x()
    r_y = obj.r_y()
    a_x = obj.local_x_axis()
    a_y = obj.local_y_axis()
    pt1 = center + r_x * a_x + r_y * a_y
    pt2 = center - r_x * a_x + r_y * a_y
    pt3 = center - r_x * a_x - r_y * a_y
    pt4 = center + r_x * a_x - r_y * a_y

    vertices = [(pt1[0], pt1[1]),
                (pt2[0], pt2[1]),
                (pt3[0], pt3[1]),
                (pt4[0], pt4[1])]

    draw_polygon_as_patch(vertices, ax,
                          zorder=zorder,
                          facecolor=facecolor,
                          edgecolor=edgecolor,
                          lw=0.5)


def draw_collision_triangle(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'triangle', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'triangle', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'triangle', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for triangle. Called through:")
        print(call_stack)
    v = obj.vertices()
    vertices = [(v[0][0], v[0][1]),
                (v[1][0], v[1][1]),
                (v[2][0], v[2][1])]

    draw_polygon_as_patch(vertices, ax, zorder=zorder, facecolor=facecolor,
                          edgecolor=edgecolor, lw=0.5)


def draw_collision_circle(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'circle', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'circle', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'circle', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for circle. Called through:")
        print(call_stack)

    patch = matplotlib.patches.Ellipse([obj.x(), obj.y()], 2 * obj.r(),
                                       2 * obj.r(), zorder=zorder,
                                       facecolor=facecolor,
                                       edgecolor=edgecolor, lw=0.5)
    ax.add_patch(patch)


def draw_collision_timevariantcollisionobject(obj, ax, draw_params, draw_func,
                                              call_stack):
    call_stack = tuple(list(call_stack) + ['time_variant_obstacle'])
    for i in range(obj.time_start_idx(), obj.time_end_idx() + 1):
        tmp = obj.get_obstacle_at_time(i)
        if "time_to_color" in draw_params.keys():
            draw_params = draw_params.copy()
            draw_params['color'] = draw_params["time_to_color"].to_rgba(i)
            fvks.visualization.draw_dispatch.draw_object(obj, ax, draw_params,
                                                         draw_func, call_stack)
        else:
            fvks.visualization.draw_dispatch.draw_object(tmp, ax, draw_params,
                                                         draw_func, call_stack)


def draw_collision_shapegroup(obj, ax, draw_params, draw_func, call_stack):
    call_stack = tuple(list(call_stack) + ['shape_group'])
    for o in obj.unpack():
        fvks.visualization.draw_dispatch.draw_object(o, ax, draw_params,
                                                     draw_func, call_stack)


def draw_collision_polygon(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'zorder'),
            ('collision', 'zorder'))
        draw_mesh = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'draw_mesh'))
    except KeyError:
        print("Cannot find stylesheet for polygon. Called through:")
        print(call_stack)
    call_stack = tuple(list(call_stack) + ['polygon'])
    if draw_mesh:
        for o in obj.get_triangle_mesh():
            fvks.visualization.draw_dispatch.draw_object(o, ax, draw_params,
                                                        draw_func, call_stack)
    else:
        draw_polygon_as_patch(obj.get_vertices(), ax, zorder=zorder, facecolor=facecolor,
                              edgecolor=edgecolor, lw=0.5)


def draw_collision_collisionchecker(obj, ax, draw_params, draw_func,
                                    call_stack):
    call_stack = tuple(list(call_stack) + ['collision_checker'])
    for o in obj.get_obstacles():
        fvks.visualization.draw_dispatch.draw_object(o, ax, draw_params,
                                                     draw_func, call_stack)


def draw_collision_boundingvolume(obj, ax, draw_params, draw_func, call_stack):
    try:
        facecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'boundingvolume', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'boundingvolume', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = fvks.visualization.draw_dispatch.retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'boundingvolume', 'zorder'),
            ('collision', 'zorder'))
        draw_recursively = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('collision', 'boundingvolume', 'draw_recursively'))
        draw_contained_obj = fvks.visualization.draw_dispatch.retrieve_value(
            draw_params, call_stack,
            ('collision', 'boundingvolume',
             'draw_contained_collision_object'))
    except KeyError:
        print("Cannot find stylesheet for bounding volume. Called through:")
        print(call_stack)

    vertices = [(obj.min_x(), obj.min_y()),  # left, bottom
                (obj.min_x(), obj.max_y()),  # left, top
                (obj.max_x(), obj.max_y()),  # right, top
                (obj.max_x(), obj.min_y())]  # right, bottom

    draw_polygon_as_patch(vertices, ax, zorder=zorder,
                          facecolor=facecolor,
                          edgecolor=edgecolor, lw=0.5)

    if draw_recursively:
        for c in obj.get_children_bounding_volumes():
            fvks.visualization.draw_dispatch.draw_object(c, ax, draw_params,
                                                         draw_func, call_stack)
    if draw_contained_obj:
        call_stack = tuple(list(call_stack) + ['bounding_volume'])
        for c in obj.get_contained_collision_objects():
            fvks.visualization.draw_dispatch.draw_object(c, ax, draw_params,
                                                         draw_func, call_stack)


draw_func_dict = {pyfvks.collision.RectAABB: draw_collision_rectaabb,
                  pyfvks.collision.RectOBB: draw_collision_rectobb,
                  pyfvks.collision.Circle: draw_collision_circle,
                  pyfvks.collision.Triangle: draw_collision_triangle,
                  pyfvks.collision.Trajectory:
                      draw_collision_timevariantcollisionobject,
                  pyfvks.collision.TimeVariantCollisionObject:
                      draw_collision_timevariantcollisionobject,
                  pyfvks.collision.ShapeGroup: draw_collision_shapegroup,
                  pyfvks.collision.Polygon: draw_collision_polygon,
                  pyfvks.collision.CollisionChecker:
                      draw_collision_collisionchecker,
                  pyfvks.collision.Point: draw_collision_point,
                  pyfvks.collision.BoundingVolume:
                      draw_collision_boundingvolume}
