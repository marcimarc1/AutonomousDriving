import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches

import pyfvks.collision

import fvks.scenario
import fvks.scenario.lanelet
import fvks.visualization.draw_dispatch
import fvks.planning.planning

import fvks.visualization.draw_dispatch
import fvks.geometry.transform
from .util import draw_polygon_as_patch


def draw_planning_task(obj, ax, draw_params, draw_func, call_stack):
    for p in obj.planning_problems:
        draw_planning_problem(p, ax, draw_params, draw_func, call_stack)


def draw_planning_problem(obj, ax, draw_params, draw_func, call_stack):
    draw_intital_state(obj.initial_state, ax, draw_params, draw_func, call_stack)
    draw_goal_region(obj.goal, ax, draw_params, draw_func, call_stack)


def draw_intital_state(obj, ax, draw_params, draw_func, call_stack):
    ax.plot(obj.position[0], obj.position[1], 'o', color='#66cdaa', zorder=20)
    ax.annotate('initial position', xy=(obj.position[0], obj.position[1]), textcoords='data', zorder=50)


def draw_goal_region(obj, ax, draw_params, draw_func, call_stack):
    for goal_state in obj.state_list:
        draw_single_goal_state(goal_state, ax, draw_params, draw_func, call_stack)


def draw_single_goal_state(obj, ax, draw_params, draw_func, call_stack):
    if type(obj.position) == list:
        for pos in obj.position:
            draw_position(pos, ax, draw_params, draw_func, call_stack)
    else:
        draw_position(obj.position, ax, draw_params, draw_func, call_stack)


def draw_position(pos, ax, draw_params, draw_func, call_stack):
    if (type(pos) == pyfvks.collision.Point or
        type(pos) == pyfvks.collision.RectOBB or
        type(pos) == pyfvks.collision.Circle or
        type(pos) == pyfvks.collision.Triangle or
        type(pos) == pyfvks.collision.ShapeGroup):
        fvks.visualization.draw_dispatch.draw_object(pos, ax, draw_params, draw_func)
    elif type(pos) == fvks.scenario.lanelet.Lanelet:
        bound = np.vstack((pos.left_vertices,
                           np.flipud(pos.right_vertices)))
        bound = np.vstack((bound, bound[0]))
        draw_polygon_as_patch(bound, ax, zorder=10, facecolor='green')
        ax.plot(bound[:, 0], bound[:, 1], color='#000000', lw=0.5, zorder=20)[0]
    else:
        raise Exception()


draw_func_dict = {fvks.planning.planning.GoalRegion: draw_goal_region,
                  fvks.planning.planning.PlanningProblem: draw_planning_problem,
                  fvks.planning.planning.PlanningTask: draw_planning_task}
