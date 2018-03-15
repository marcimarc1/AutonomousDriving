import os
import numpy as np
import pickle
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from fvks.scenario.commonroad.file_reader import CommonRoadFileReader
from fvks.visualization.draw_dispatch import draw_object
from traj_planner.BerthaTrajectoryPlanning.OneRoadTestEnv.collision_model_bertha import *

# pickle_in = open('trajectory.p', 'rb')
# trajectory = pickle.load(pickle_in)
# pickle_in.close()
# states = np.array([s['position'] for s in trajectory], dtype='float64')
# new_tr = np.array(states[:25])
number_of_samples=35
xs = np.linspace(0.5, 80, 30)
ys = np.linspace(1.5, 3., 30)
states = np.array(list(zip(xs, ys)))
new_tr = states
final_state = np.array([80, 4])
_x = np.linspace(states[0][0], final_state[0], num=number_of_samples)
_y = np.linspace(2, 5, num=number_of_samples)
new_tr = np.column_stack((_x, _y))
# plt.figure()
# plt.plot(new_tr[:,0], new_tr[:,1], 'x')
# plt.show()
# exit()
# print(trajectory[0])
# exit()

h = 0.5
v_des = 10

v_radius = 2  # TODO: get rid of this in the future
kmax = np.pi / 4.
amax = 8
vmax = 15
init_vel = 1
l_left = 6
l_right = 0


def euclid_dist(a, b, axis=1):
    return np.linalg.norm((a - b), axis=axis)


def grad(x, nth=1, n=2, h=1):
    ds = [{
        'grad': x,
        'grad_mag': x
    }]
    for i in range(nth):
        _x = ds[-1]['grad_mag']
        grad = np.array(np.gradient(_x))
        grad_mag = np.array(grad)
        # print(grad.shape)
        if (n > 1):
            grad_mag = np.linalg.norm(grad, axis=0)
        ds.append({
            'grad': grad,
            'grad_mag': grad_mag
        })
    return ds[-1]['grad_mag'], ds[-1]['grad'], ds


def psi(x):
    return np.arctan2(np.gradient(x[:, 1]), np.gradient(x[:, 0]))


def kappa(x):
    _, _, dxs = grad(x[:, 0], 2, n=1)
    _, _, dys = grad(x[:, 1], 2, n=1)
    den = np.power(np.add(np.power(dxs[1]['grad'], 2), np.power(dys[1]['grad'], 2)), 1 / 3)
    den[den==0] = -1e-15
    num = np.subtract(np.multiply(dxs[1]['grad'], dys[2]['grad']), np.multiply(dys[1]['grad_mag'], dxs[2]['grad_mag']))
    return np.divide(num, den)


def cons_left_right(x):
    # TODO: get these points from the scenerio
    left = np.array(x)
    right = np.array(x)
    left[:, 1] = l_left
    right[:, 1] = l_right
    return left, right


def dist_to_f(x):
    print((x - final_state).shape)
    return np.linalg.norm((x - final_state))


def calculatevdes(x):
    left, right = cons_left_right(x)
    x_norm = np.linalg.norm(x, axis=1)
    delta_d_l = np.divide(np.subtract(x, left), x_norm[:, np.newaxis])
    delta_d_r = np.divide(np.subtract(x, left), x_norm[:, np.newaxis])
    return np.matmul(v_des * np.array([[0, -1], [1, 0]], dtype='float64'),
                     0.5 * (np.add(delta_d_l, delta_d_r).T)).T


def joffs(x):
    left, right = cons_left_right(x)
    dleft = euclid_dist(x, left)
    dright = euclid_dist(x, right)
    return np.abs(0.5 * (dleft - dright)) ** 2


def jvel(x):
    d, _, _ = grad(x)
    udes = calculatevdes(x)
    return np.linalg.norm(np.subtract(calculatevdes(x), d), axis=1)


def jacc(x):
    d, _, _ = grad(x, 2)
    return np.power(np.abs(np.linalg.norm(d, axis=1)), 2)


def jjerk(x):
    d, _, _ = grad(x, 3)
    return np.power(np.abs(np.linalg.norm(d, axis=1)), 2)


def jyawr(x):
    d, _, _ = grad(psi(x), nth=1, n=1)
    return np.power(d, 2)


nl_opt = []
_js = []

w = np.array([1., 1, 1.2, .10, 1])


def bertha_obj(x):
    x = np.reshape(x, (len(x) // 2, 2))  # scipy flattens array

    nl_opt.append(x)
    # w = np.array([3, 10., 100., 0.9, 0.5])
    _joffs = joffs(x)
    # print(_joffs.shape)
    _jvel = jvel(x)
    # print(_jvel.shape)
    _jacc = jacc(x)
    # # print(_jacc.shape)
    _jjerk = jjerk(x)
    # # print(_jjerk.shape)
    _jyawr = jyawr(x)
    # _jdist = dist_to_f(x)
    j = np.array([_joffs, _jvel, _jacc, _jjerk, _jyawr])
    _js.append(j)
    cost = np.sum(np.dot(w.T, j))
    return cost


def kappa_constraint_left(x):
    x = np.reshape(x, (len(x) // 2, 2))
    # print(x.shape)
    # −κmax ≤ κ(t) ≤ κmax.
    # TODO: get κmax from vehicle model.
    kap = kappa(x)
    # kap[kap<=kmax] = 0.
    # kap[kap>kmax] = kmax - kap[kap>kmax]
    kap = kmax - kap
    return kap
    # x = np.reshape(x, (len(x) // 2, 2))
    # # print(x.shape)
    # # −κmax ≤ κ(t) ≤ κmax.
    # # TODO: get κmax from vehicle model.
    # return (kmax - np.abs(kappa(x))).flatten()


def kappa_constraint_right(x):
    x = np.reshape(x, (len(x) // 2, 2))
    # print(x.shape)
    # −κmax ≤ κ(t) ≤ κmax.
    # TODO: get κmax from vehicle model.
    kap = kappa(x)
    # kap[kap<=kmax] = 0.
    # kap[kap>kmax] = kmax - kap[kap>kmax]
    kap = kap + kmax
    return kap
    # x = np.reshape(x, (len(x) // 2, 2))
    # # print(x.shape)
    # # −κmax ≤ κ(t) ≤ κmax.
    # # TODO: get κmax from vehicle model.
    # return (kmax - np.abs(kappa(x))).flatten()


def acc_constraint(x):
    x = np.reshape(x, (len(x) // 2, 2))
    # TODO: get acc from scenario
    acc, _, _ = grad(x, nth=2, n=2)
    return (amax - acc).flatten()


def dist_to_corridor(x):
    x = np.reshape(x, (len(x) // 2, 2))
    dist = joffs(x)
    # TODO: get acc from scenario
    return dist - 100

    e = euclid_dist(x, c) - 5
    # dist = e  # np.append(e,e2)


def vel_const(x):
    x = np.reshape(x, (len(x) // 2, 2))
    d, _, _ = grad(x)
    return vmax - np.linalg.norm(d, axis=1)

    e = euclid_dist(x, c) - 2.5
    dist = e  # np.append(e,e2)


def ini_vel_const(x):
    x = np.reshape(x, (len(x) // 2, 2))
    d, _, _ = grad(x)
    grad_mag = np.linalg.norm(d, axis=1)
    return init_vel - grad_mag[0]


def collision_const(x):
    x = np.reshape(x, (len(x) // 2, 2))
    c = np.array([10, 2.5] * (len(x)))
    c = np.reshape(c, (len(c) // 2, 2))

    e = euclid_dist(x, c) - 1.5
    dist = e  # np.append(e,e2)

    return dist

def collision_const1(x):
    x = np.reshape(x, (len(x) // 2, 2))
    c = np.array([30, 2.5] * (len(x)))
    c = np.reshape(c, (len(c) // 2, 2))

    e = euclid_dist(x, c) - 2.
    dist = e  # np.append(e,e2)

    return dist

def collision_const2(x):
    x = np.reshape(x, (len(x) // 2, 2))
    c = np.array([50, 3] * (len(x)))
    c = np.reshape(c, (len(c) // 2, 2))

    e = euclid_dist(x, c) - 1.5
    dist = e  # np.append(e,e2)

    return dist

vehicle = Vehicle(4, 3, 2, 1.1)


def collision_model(x):
    x = np.reshape(x, (len(x) // 2, 2))
    v = vehicle.alignment(x, psi(x))
    v = np.array(v)
    v = np.reshape(v, (number_of_samples * 4, 2))
    c = np.array([10, 1.5] * (len(v)))
    c = np.reshape(c, (len(c) // 2, 2))
    e = euclid_dist(v, c)
    # print(np.shape(e))
    dist = e - 1.5 - vehicle.radius-0.1
    # dist = np.append([],dist[dist<=0])
    # if 0==len(dist):
    #     dist = np.append(dist,[0])
    # print(e-2.5)
    return dist.flatten()


def collision_model1(x):
    x = np.reshape(x, (len(x) // 2, 2))
    v = vehicle.alignment(x, psi(x))
    v = np.array(v)
    v = np.reshape(v, (number_of_samples * 4, 2))
    c = np.array([20, 5.5] * (len(v)))
    c = np.reshape(c, (len(c) // 2, 2))
    e = euclid_dist(v, c)
    # print(np.shape(e))
    dist = e - 1.5 - vehicle.radius-0.1
    # dist = np.append([],dist[dist<=0])
    # if 0==len(dist):
    #     dist = np.append(dist,[0])
    # print(e-2.5)
    return dist.flatten()

def collision_model2(x):
    x = np.reshape(x, (len(x) // 2, 2))
    v = vehicle.alignment(x, psi(x))
    v = np.array(v)
    v = np.reshape(v, (number_of_samples * 4, 2))
    c = np.array([33, 1.5] * (len(v)))
    c = np.reshape(c, (len(c) // 2, 2))
    e = euclid_dist(v, c)
    # print(np.shape(e))
    dist = e - 1.5 - vehicle.radius-0.1
    # dist = np.append([],dist[dist<=0])
    # if 0==len(dist):
    #     dist = np.append(dist,[0])
    # print(e-2.5)
    return dist.flatten()
#
def collision_model3(x):
    x = np.reshape(x, (len(x) // 2, 2))
    v = vehicle.alignment(x, psi(x))
    v = np.array(v)
    v = np.reshape(v, (number_of_samples * 4, 2))
    c = np.array([55, 5.5] * (len(v)))
    c = np.reshape(c, (len(c) // 2, 2))
    e = euclid_dist(v, c)
    # print(np.shape(e))
    dist = e - 1.5 - vehicle.radius
    # dist = np.append([],dist[dist<=0])
    # if 0==len(dist):
    #     dist = np.append(dist,[0])
    # print(e-2.5)
    return dist.flatten()
#
def collision_model4(x):
    x = np.reshape(x, (len(x) // 2, 2))
    v = vehicle.alignment(x, psi(x))
    v = np.array(v)
    v = np.reshape(v, (number_of_samples * 4, 2))
    c = np.array([52, 5.5] * (len(v)))
    c = np.reshape(c, (len(c) // 2, 2))
    e = euclid_dist(v, c)
    # print(np.shape(e))
    dist = e - 1.5 - vehicle.radius-0.1
    # dist = np.append([],dist[dist<=0])
    # if 0==len(dist):
    #     dist = np.append(dist,[0])
    # print(e-2.5)
    return dist.flatten()


# def boundary_constraints:



# bnd = (1.0,5.0)
# bnds= (bnd,bnd,bnd,bnd)
x_bound = (0, 90)
y_bound = (1 , 5)
bnds = []
for i in new_tr:
    bnds.append(x_bound)
    bnds.append(y_bound)

# bnds = np.tile([x_bound, y_bound], (len(new_tr), 1))
con1 = {'type': 'ineq', 'fun': kappa_constraint_left}
con1_1 = {'type': 'ineq', 'fun': kappa_constraint_right}
con2 = {'type': 'ineq', 'fun': acc_constraint}
con3 = {'type': 'eq', 'fun': lambda x: np.subtract(states[0], x[0:2])}
con4 = {'type': 'eq', 'fun': lambda x: np.subtract(final_state, x[-2:])}
con7 = {'type': 'ineq', 'fun': vel_const}
con8 = {'type': 'eq', 'fun': ini_vel_const}
con9 = {'type': 'ineq', 'fun': collision_const}
con14 = {'type': 'ineq', 'fun': collision_const1}
con15 = {'type': 'ineq', 'fun': collision_const2}
con10 = {'type': 'ineq', 'fun': collision_model}
con11 = {'type': 'ineq', 'fun': collision_model1}
con12 = {'type': 'ineq', 'fun': collision_model2}
con13 = {'type': 'ineq', 'fun': collision_model3}
# con14 = {'type': 'ineq', 'fun': collision_model4}
# con5 = {'type':'ineq', 'fun': dist_to_corridor}
# con4 = {'type':'ineq', 'fun': lambda x: (jvel(np.reshape(x, (51,2)) )) }

cons = [con1, con1_1, con2, con3, con4, con7, con8, con10, con11, con12]#,con10, con11, con12]#, con13]#, con14]  # , con11]
x0 = np.array(new_tr).flatten()
sol = minimize(bertha_obj, x0, method='SLSQP', bounds=bnds, constraints=cons, options = {'maxiter': 400, 'ftol':1e-3})
print(sol)
xf = np.reshape(sol.x, (len(x0) // 2, 2))
print(np.array(nl_opt).shape)
plt.figure(figsize=(25, 10))
font = {'family': 'serif',
        'color': 'darkred',
        'weight': 'normal',
        'size': 16,
        }
w_new = np.array2string(w)
# plt.title('Weights: ' + w_new, fontdict=font)
x0_c = np.reshape(x0, (len(x0) // 2, 2))
plt.plot(x0_c[:, 0], x0_c[:, 1], '-x')
plt.plot(xf[:, 0], xf[:, 1], '-o')

# _js=np.array(_js)
# print(_js.shape)
# plt.plot(_js[:,0], np.arange(len(_js)), 'x')

# plt.plot(nl_opt[1][:,0], nl_opt[1][:,1], 'x')
# plt.plot(nl_opt[2][:,0], nl_opt[2][:,1], 'x')

# plot scnerio
filename = os.getcwd() + "/OneRoad.xml"  # /traj_planner/BerthaTrajectoryPlanning/OneRoadTestEnv
scenario, planning_task = CommonRoadFileReader(filename).open()

draw_object(scenario)
# draw_object(planning_task)
ss=[]
ss.append(plt.Circle((10, 1.5), 1.5))
ss.append(plt.Circle((20, 5.5),1.5))
ss.append(plt.Circle((33, 1.5), 1.5))
#ss.append(plt.Circle((55,5.5), 1.5))
#ss.append(plt.Circle((50, 5.5), 1.5))

ax = plt.gca()
for i in ss:
    ax.add_artist(i)
v = vehicle.alignment(xf, psi(xf))
v = np.array(v)
v = np.reshape(v, (number_of_samples * 4, 2))
c = []
for i in v:
    ax.plot(i[0], i[1])
    # for j in range(len(i)):
    c.append(plt.Circle((i[0], i[1]), radius=1.1, color='g', fill=False))

for i in c:
    ax.add_artist(i)
    ax.plot()

plt.gca().set_aspect('equal')
plt.axis('off')
plt.savefig(os.getcwd() + "/solution_pic/final/vel.svg")
plt.show()
# plt.show()
# plt.show()
# plt.show()
