import os
import matplotlib.pyplot as plt
import numpy as np
import math

from bertha.helpers.helpermethods import reshaper
from fvks.visualization.draw_dispatch import draw_object
from drivearea import morphingcommonroad, bounds, goal_bounds, constraint_interpolation  # , constraint_interpolation
from scipy.optimize import minimize
from collision_model_bertha import Vehicle
import numexpr as ne
from bertha.environment.obstacle import *
from bertha.helpers.distance import *
from datetime import datetime
import warnings

#warnings.filterwarnings('error')

cwd = os.getcwd()
cwd = os.getcwd() + '/scenarios/NGSIM/US101/NGSIM_US101_0.xml'
# cwd = cwd - "OneRoadTestEnv"
cwd = cwd.replace('OneRoadTestEnv/', '')  # sub("OneRoadTestEnv",'')
# filename = os.getcwd() + "/automated-driving/commonroad/scenarios/NGSIM/C-NGSIM_US101_0.xml"
new_sim = morphingcommonroad(cwd)

planning_task = new_sim.get_planning_task()
dynamic_bounds = new_sim.dynamic_obstacles_boundary()
dt = new_sim.scenario.dt
timesteps = new_sim.get_time_by_dynamic_obstacles()

pointsright, pointsleft = new_sim.generating_driving_corridor



# pseudo_distance object to compute the distance
pseudo_distance = PsuedoDistance()




# states = np.array([s*dt for s in range(timesteps)], dtype='float64')
# new_tr = np.array(states[:25])
# get_final_state() returns the center of the rectOBB
# funkel = new_sim.get_final_state()
final_state1 = planning_task.planning_problems[0].goal
final_state = final_state1.state_list[0][0]
#final_state2 = final_state.center()
time_of_final_state = final_state1.state_list[0].time
xs = np.linspace(-31., 100, timesteps)
ys = np.linspace(3., -10., timesteps)
states = np.array(list(zip(xs, ys)))
new_tr = states

limitleft, limitright = constraint_interpolation(new_tr, pointsleft, pointsright)

obstacles_left = StaticObstacleHolder(pointsleft, dt)
obstacles_right = StaticObstacleHolder(pointsright, dt, True)



v_des = new_sim.get_speed_limit()

# some values to get from the vehicle model or simulation
v_radius = 0.5  # TODO: get rid of this in the future
kmax = np.pi / 1000.
amax = 5.
vmax = 28.
init_vel = 10.
car_length = 4
car_width = 2
r_veh = math.sqrt(2)
vehicle_model = Vehicle(4, car_length, car_width, r_veh)

'''Begin of the simulation (code of optimization_test)'''


# this is for driving corridor
def euclid_dist(a, b, axis=1):
    if(np.all(a.shape == b.shape)):
        dist = np.linalg.norm(a-b, axis=axis)
    else:
        B = b[:, np.newaxis]
        sub = B-a
        dists = np.linalg.norm(sub, axis=2)
        dist = np.amin(dists,axis=0).T
    return dist


# this is for distance to obstacles
# def get_distance_from_obst(veh_pos, obst_points):
#    return pseudo_distance.compute_distance(veh_pos, obst_points)



def grad(x, nth=1, n=2, h=1):
    ds = [{
        'grad': x,
        'grad_mag': x
    }]
    for i in range(nth):
        _x = np.array(ds[-1]['grad_mag'])
        grad = np.array(np.gradient(_x, dt))
        grad_mag = np.array(grad)
        # print(grad.shape)
        if (n > 1):
            grad_mag = np.linalg.norm(grad, axis=0)
        ds.append({
            'grad': grad,
            'grad_mag': grad_mag
        })
    return ds[-1]['grad_mag'], ds[-1]['grad'], ds


# tangent angle
def psi(x):
    return np.arctan2(np.gradient(x[:, 1],dt), np.gradient(x[:, 0],dt))


# curvature of the trajectory
def kappa(x):
    _, _, dxs = grad(x[:, 0], 2, n=1)
    _, _, dys = grad(x[:, 1], 2, n=1)
    den = np.power(np.add(np.power(dxs[1]['grad'], 2), np.power(dys[1]['grad'], 2)), 1 / 3)+1.e-20
    num = np.subtract(np.multiply(dxs[1]['grad'], dys[2]['grad']), np.multiply(dys[1]['grad'], dxs[2]['grad']))
    return np.divide(num, den)


# def cons_left_right(x):
#     left = np.array(x)
#     right = np.array(x)
#     left[:, 1] = l_left
#     right[:, 1] = l_right
#     return left, right


def dist_to_f(x):
    print((x - final_state).shape)
    return np.linalg.norm((x - final_state))


def calculatevdes(x, ddleft, ddright):
    return np.matmul(v_des * np.array([[0, -1], [1, 0]], dtype='float64'),
                     0.5 * (np.add(ddleft, ddright).T)).T


def joffs(x, dleft, dright):
    return np.abs(0.5 * (dleft - dright)) ** 2

# def joffs(x):
#     dleft = euclid_dist(x, pointsleft) 
#     dright = euclid_dist(x, pointsright) 
#     return np.abs(0.5 * (dleft + dright)) ** 2

# def joffs(x):
#     turned_point = vehicle_model.alignment(x, psi(x))
#     distance_to_left, distance_to_right = pseudo_distance.compute_distance_to_driver_corridors(turned_point, pointsleft,
#                                                                                                pointsright)
#     distance_to_left -= v_des
#     distance_to_right -= v_des
#     return np.abs(0.5 * (distance_to_left - distance_to_right)) ** 2

def jvel(x, ddleft, ddright):
    d, _, _ = grad(x)
    udes = calculatevdes(x, ddleft, ddright)
    return np.linalg.norm(np.subtract(udes, d), axis=1)


def jacc(x):
    d, _, _ = grad(x, 2)
    return np.power(np.abs(np.linalg.norm(d, axis=1)), 2)


def jjerk(x):
    d, _, _ = grad(x, 3)
    return np.power(np.abs(np.linalg.norm(d, axis=1)), 2)


def jyawr(x):
    d, _, _ = grad(psi(x), nth=1, n=1)
    return np.power(d, 2)

def normalize(x):
    return ((x - x.min()) / (x.max() - x.min()))

def normalize_std(x):
    return (x - np.mean(x)) / np.std(x)

nl_opt = []
_js = []
# weights = offset, velocity, acceleration, jerk, jawrate
w = np.array([0.5, 0.1, 0.1, 1., .1])

def bertha_obj(x):
    x = np.reshape(x, (len(x) // 2, 2))  # scipy flattens array
    nl_opt.append(x)
    dist_left, n_left = pseudo_distance.compute_distance_static_obstacles(x, obstacles_left)
    dist_right, n_right = pseudo_distance.compute_distance_static_obstacles(x, obstacles_right)
    _joffs = (joffs(x, dist_left, dist_right))
    _jvel = (jvel(x, n_left / np.abs(dist_left)[:,None], n_right / np.abs(dist_right)[:,None]))
    _jacc = normalize(jacc(x))
    _jjerk = normalize(jjerk(x))
    _jyawr = (jyawr(x))
    j = np.array([_joffs, _jvel, _jacc, _jjerk, _jyawr])
    #print(j)
    _js.append(j)
    cost = np.sum(np.dot(w.T, j))/np.sum(w)
    print(cost)
    return cost




def kappa_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    # print(x.shape)
    # −κmax ≤ κ(t) ≤ κmax.
    # TODO: get κmax from vehicle model.
    kap = kappa(x)
    kap = kmax - kap
    return kap

def kappa_constraint_neg(x):
    x = np.reshape(x, (len(x) // 2, 2))
    # print(x.shape)
    # −κmax ≤ κ(t) ≤ κmax.
    # TODO: get κmax from vehicle model.
    kap = kappa(x)
    # kap[kap<=kmax] = 0.
    # kap[kap>kmax] = kmax - kap[kap>kmax]
    kap = kap + kmax
    return kap

def acc_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    # TODO: get acc from scenario -> there is no constraint in the scenario -> see vehicle model
    acc, _, _ = grad(x, nth=2, n=2)
    acc = np.linalg.norm(acc,axis=1)
    # print(acc.shape)
    acc_con = amax**2 - acc**2
    return acc_con


def drive_between_corridor_left(x):
    x = np.reshape(x, (len(x) // 2, 2))
    dist_left, _ = pseudo_distance.compute_distance_static_obstacles(x, obstacles_left)
    return dist_left

def drive_between_corridor_right(x):
    x = np.reshape(x, (len(x) // 2, 2))
    dist_right, n_right = pseudo_distance.compute_distance_static_obstacles(x, obstacles_right)
    return dist_right


def vel_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    d, _, _ = grad(x)
    vel = np.linalg.norm(d)
    return (vmax - vel)


def collision_constraint(x):
    x = reshaper(x)
    x_n = np.repeat(x, len(x), axis=0)
    d_b_shape = np.array(dynamic_bounds).shape
    x_n = np.repeat(x_n, d_b_shape[2], axis=0)
    x_n = np.reshape(np.tile(x_n, (len(dynamic_bounds), 1)),
                     (len(dynamic_bounds), (len(x) ** 2) * d_b_shape[2], 2))
    d_b = np.reshape(dynamic_bounds,
                     (len(dynamic_bounds), d_b_shape[1] * d_b_shape[2], 2))
    d_b = np.tile(d_b, (1, len(x), 1))
    dists = np.linalg.norm(ne.evaluate('d_b - x_n'), axis=2) - 1.5 - 1.5
    return dists.flatten()

def ini_vel_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    d, _, _ = grad(x)
    grad_mag = np.linalg.norm(d, axis=1)
    return init_vel - grad_mag[1]

def dist_to_goal(x):
    x = reshaper(x)
    #print(-1.*euclid_dist(np.array([1.10e+02,  0.11742588e+00]), x[-1], axis=0))
    return -1.*euclid_dist(np.array([100,-1]), x[-1], axis=0)

def streetboundleft_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    return x[:,1]- limitleft[:,1]

def streetboundright_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    return x[:,1] +limitright[:,1]
'''Start of the optimization /minimization'''
# bnd = (1.0,5.0)
# bnds= (bnd,bnd,bnd,bnd)

lower_x, upper_x, lower_y, upper_y = bounds(pointsleft, pointsright)
x_bound = (lower_x, upper_x)
y_bound = (lower_y, upper_y)
bnds = []
for i in new_tr:
    bnds.append(x_bound)
    bnds.append(y_bound)
# setting bounds for goal region.
#x_goal, y_goal = goal_bounds(final_state)
#bnds[-1] = x_goal
#bnds[-2] = y_goal

# bnds = np.tile([x_bound, y_bound], (len(new_tr), 1))
con1 = {'type': 'ineq', 'fun': kappa_constraint}
con1_1 = {'type': 'ineq', 'fun': kappa_constraint_neg}
con1_2 = {'type': 'ineq', 'fun': acc_constraint}
con3 = {'type': 'eq', 'fun': lambda x: np.subtract(states[0], np.reshape(x, (len(x)//2, 2))[0])}
con4 = {'type': 'ineq', 'fun': dist_to_goal}
con5 = {'type':'ineq', 'fun': drive_between_corridor_left}
con5_1 = {'type':'ineq', 'fun': drive_between_corridor_right}
con7 = {'type': 'ineq', 'fun': vel_constraint}
con8 = {'type': 'eq', 'fun': ini_vel_constraint}
con9 = {'type': 'ineq', 'fun': collision_constraint}
#con10 = {'type': 'ineq', 'fun':streetboundleft_constraint}
#con11 = {'type': 'ineq', 'fun':streetboundright_constraint}
# con4 = {'type':'ineq', 'fun': lambda x: (jvel(np.reshape(x, (51,2)) )) }

cons = [con1, con1_1, con1_2, con3, con5, con5_1, con4, con7, con8]
x0 = np.array(new_tr).flatten()
sol = minimize(bertha_obj, x0, method='SLSQP', constraints=cons, options={'disp': True})
# sol = minimize(bertha_obj, sol.x, method='SLSQP', bounds=bnds, constraints=cons)
# sol = minimize(bertha_obj, sol.x, method='SLSQP', bounds=bnds, constraints=cons)
print(sol)
xf = np.reshape(sol.x, (len(x0) // 2, 2))
print(np.array(nl_opt).shape)

plt.figure()
font = {'family': 'serif',
        'color': 'darkred',
        'weight': 'normal',
        'size': 16,
        }
w_new = np.array2string(w)
plt.title('Weights: ' + w_new, fontdict=font)
x0_c = np.reshape(x0, (len(x0)//2, 2))
plt.plot(x0_c[:, 0], x0_c[:, 1], '-x')
plt.plot(xf[:, 0], xf[:, 1], '-o')
plt.plot(pointsleft[:, 0], pointsleft[:,1], '--')
plt.plot(pointsright[:, 0], pointsright[:,1], '--')
draw_object(new_sim.scenario)
draw_object(final_state)
plt.gca().set_aspect('equal')
plt.savefig(os.getcwd() + "/solution_pic/" + datetime.now().strftime("%I:%M%p on %B %d, %Y").replace(" ", "") + ".svg")
plt.show()
