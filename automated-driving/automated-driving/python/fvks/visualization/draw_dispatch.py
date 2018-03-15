import sys
import matplotlib.pyplot as plt

try:
    import fvks.visualization.pyfvks_collision
except ImportError:
    pass

try:
    import fvks.visualization.pyfvks_reach
except ImportError:
    pass

try:
    import fvks.visualization.scenario
except ImportError:
    pass

try:
    import fvks.visualization.planning
except ImportError:
	pass

try:
    import fvks.visualization.pyfvks_geometry
except ImportError:
    pass

# import fvks.visualization.corridors

def create_drawers_dict():
    draw_func = {}
    if 'fvks.visualization.pyfvks_collision' in sys.modules.keys():
        draw_func.update(fvks.visualization.pyfvks_collision.draw_func_dict)
    if 'fvks.visualization.pyfvks_reach' in sys.modules.keys():
        draw_func.update(fvks.visualization.pyfvks_reach.draw_func_dict)
    if 'fvks.visualization.pyfvks_geometry' in sys.modules.keys():
        draw_func.update(fvks.visualization.pyfvks_geometry.draw_func_dict)
    if 'fvks.visualization.scenario' in sys.modules.keys():
        draw_func.update(fvks.visualization.scenario.draw_func_dict)
    if 'fvks.visualization.planning' in sys.modules.keys():
        draw_func.update(fvks.visualization.planning.draw_func_dict)

    return draw_func


def create_default_draw_params():
    draw_params = {'time_begin:': 0, 'time_end': 50}
    if 'fvks.visualization.pyfvks_collision' in sys.modules.keys():
        draw_params.update(
            fvks.visualization.pyfvks_collision.create_default_draw_params())
    if 'fvks.visualization.pyfvks_geometry' in sys.modules.keys():
        draw_params.update(
            fvks.visualization.pyfvks_geometry.create_default_draw_params())    
    if 'fvks.visualization.scenario' in sys.modules.keys():
        draw_params.update(
            fvks.visualization.scenario.create_default_draw_params())
    if 'fvks.visualization.pyfvks_reach' in sys.modules.keys():
        draw_params.update(
            fvks.visualization.pyfvks_reach.create_default_draw_params())
    return draw_params


default_draw_params = create_default_draw_params()


def retrieve_value_by_path(style_sheet_caller, value_path):
    c_dict = style_sheet_caller
    for value_element in value_path[:-1]:
        try:
            c_dict = c_dict[value_element]
        except KeyError:
            raise KeyError()
    try:
        return_value = c_dict[value_path[-1]]
    except KeyError:
        raise KeyError()
    return return_value


def retrieve_value(style_sheet, call_stack, value_path):
    for idx, caller in enumerate(call_stack):
        try:
            style_sheet_caller = style_sheet[caller]
            try:
                value = retrieve_value_by_path(style_sheet_caller, value_path)
                return value
            except KeyError:
                pass
        except KeyError:
            continue

    try:
        value = retrieve_value_by_path(style_sheet['no_parent'], value_path)
        return value
    except KeyError:
        pass
    value = retrieve_value_by_path(default_draw_params, value_path)
    return value


def retrieve_alternate_value(style_sheet, call_stack, value_path_1,
                             value_path_2):
    for idx, caller in enumerate(call_stack):
        try:
            style_sheet_caller = style_sheet[caller]
            try:
                value = retrieve_value_by_path(style_sheet_caller, value_path_1)
                return value
            except KeyError:
                pass
            try:
                value = retrieve_value_by_path(style_sheet_caller, value_path_2)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    try:
        value = retrieve_value_by_path(style_sheet['no_parent'], value_path_1)
        return value
    except KeyError:
        pass
    try:
        value = retrieve_value_by_path(style_sheet['no_parent'], value_path_2)
        return value
    except KeyError:
        pass

    try:
        value = retrieve_value_by_path(default_draw_params, value_path_1)
        return value
    except KeyError:
        pass
    value = retrieve_value_by_path(default_draw_params, value_path_2)
    return value


def draw_object(obj, ax=None, draw_params=None,
                draw_func=None, call_stack=None):
    if type(obj) is list:
        for o in obj:
            draw_object(o, ax, draw_params, draw_func)
        return

    if draw_func is None:
        draw_func = create_drawers_dict()

    if draw_params is None:
        draw_params = default_draw_params

    if call_stack is None:
        call_stack = tuple()

    if ax is None:
        ax = plt.gca()

    if not type(obj) in draw_func.keys():
        print("Cannot dispatch to plot " + str(type(obj)))
    else:
        draw_func[type(obj)](obj, ax, draw_params, draw_func, call_stack)
