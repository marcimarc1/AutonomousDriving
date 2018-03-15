import os
import matplotlib.pyplot as plt
import numpy as np
import math

import time
from fvks.visualization.draw_dispatch import draw_object
from drivearea import morphingcommonroad, bounds, goal_bounds  # , constraint_interpolation
from scipy.optimize import minimize
from bertha.environment.collision_model_bertha import Vehicle
from bertha.helpers.distance import *
from bertha.environment.obstacle import *
from pylab import *
import matplotlib.patches as patches
cwd = os.getcwd()
cwd = os.getcwd() + '/scenarios/NGSIM/US101/NGSIM_US101_0.xml'
# cwd = cwd - "OneRoadTestEnv"
cwd = cwd.replace('OneRoadTestEnv/', '')  # sub("OneRoadTestEnv",'')
# filename = os.getcwd() + "/automated-driving/commonroad/scenarios/NGSIM/C-NGSIM_US101_0.xml"
new_sim = morphingcommonroad(cwd)
pseudo_distance = PsuedoDistance()


def grad(x, nth=1, n=2, h=1):
    dt = 0.1
    ds = [{
        'grad': x,
        'grad_mag': x
    }]
    for i in range(nth):
        _x = ds[-1]['grad_mag']
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

def psi(x):
    return np.arctan(np.gradient(x[:, 1])/ np.gradient(x[:, 0]))

def visualize_pseudo_dist():

    boundsright, boundsleft = new_sim.generating_driving_corridor
    lower_x, upper_x, lower_y, upper_y = bounds(pointsleft = boundsleft, pointsright= boundsright)
    xs = np.linspace(-14., 214., 50)
    ys = np.linspace(-2., 4.,  50)

    veh_matrix = np.array(list(zip(xs,ys)))
    obstacle_left = StaticObstacleHolder(boundsleft)
    dist_left, distance_vector = pseudo_distance.compute_distance_static_obstacles(veh_matrix, obstacle_left)
    #left.reshape((len(left) // 2, 2))
    plt.figure()
    plt.plot(veh_matrix[:, 0], veh_matrix[:, 1])
    plt.plot(boundsleft[:,0], boundsleft[:,1])
    plt.quiver(veh_matrix[:,0],veh_matrix[:,1], distance_vector[:,0],distance_vector[:,1])
    plt.savefig(os.getcwd() + "/solution_pic/dist_left.png")
    plt.show()

def visualize_pseudo_dist_2():
    o = np.array([[-1.,-1.], [0.,0.], [1.,0.], [2.,-1.]])
    obstacle = StaticObstacle(o)

    xs = np.linspace(-1., 2., 3)
    ys = np.linspace(-1., 2.,  3)

    veh_matrix = np.array(list(zip(xs,ys)))
    dist, nlambda, plambda= obstacle.compute_distance(veh_matrix)
    #left.reshape((len(left) // 2, 2))
    plt.figure()
    plt.plot(veh_matrix[:, 0], veh_matrix[:, 1])
    plt.plot(o[:,0], o[:,1])
    plt.quiver(plambda[:,0],plambda[:,1], nlambda[:,0],nlambda[:,1])
    plt.show()
    #plt.savefig(os.getcwd() + "/solution_pic/dist_left.png")

def visualize_vehicle_model():
    x = np.linspace(0,2 *np.pi, 30)
    sin = 5 *np.sin( 0.35 * x)
    m = Vehicle(3, 0.5,0.1,0.1)
    pos = np.array([x, sin]).T
    c = []
    time1 = time.time()
    allign = m.alignment(pos)
    time2 = time.time()
    print('function took %0.3f ms' % ((time2 - time1) * 1000.0))
    fig, ax = plt.subplots(1, 1)
    ax.set_xlim((min(x)-1, max(x)+1))
    ax.set_ylim((min(sin)-1, max(sin)+1))
    fig.show()
    for i in allign:
        ax.plot(i[:,0],i[:,1])
        for j in range(len(i[0])):
            c.append(plt.Circle((i[j,0],i[j,1]), 0.25, color='g', fill=False))
    #plt.plot(pos[:,0],pos[:,1])
    for i in c:
        ax.add_artist(i)
        ax.plot()
    #fig.savefig(os.getcwd() + "/solution_pic/sincarshape.png")
    fig.show()
    a = 2


visualize_vehicle_model()
#visualize_pseudo_dist()
def visualize_commmonroad():
    plt.figure(figsize=(25, 10))
    #traj = array([ -3.10000000e+01,   3.00000000e+00,  -2.86511755e+01,         2.93282558e+00,  -2.63013829e+01,   2.86564510e+00,        -2.39525528e+01,   2.79849298e+00,  -2.16027388e+01,         2.73133333e+00,  -1.92538878e+01,   2.66420308e+00,        -1.69040526e+01,   2.59706424e+00,  -1.45551815e+01,         2.52995588e+00,  -1.22053251e+01,   2.46283787e+00,        -9.85643706e+00,   2.39575149e+00,  -7.50656551e+00,         2.32865446e+00,  -5.15775357e+00,   2.26159272e+00,        -2.80895089e+00,   2.19454764e+00,  -4.62957851e-01,         2.12758884e+00,   1.86471913e+00,   2.06116817e+00,         4.29200653e+00,   1.99192136e+00,   6.61902251e+00,         1.92554429e+00,   8.96975650e+00,   1.85850026e+00,         1.13153157e+01,   1.79161897e+00,   1.36680827e+01,         1.72453907e+00,   1.60141869e+01,   1.65766436e+00,         1.83670217e+01,   1.59060435e+00,   2.07131911e+01,         1.52374986e+00,   2.30660735e+01,   1.45671031e+00,         2.54113850e+01,   1.38990235e+00,   2.77651378e+01,         1.32285981e+00,   3.01105016e+01,   1.25607244e+00,         3.24642746e+01,   1.18905113e+00,   3.48096646e+01,         1.12228509e+00,   3.71634576e+01,   1.05528503e+00,         3.95088719e+01,   9.88540374e-01,   4.18626853e+01,         9.21561538e-01,   4.42081229e+01,   8.54838300e-01,         4.65619568e+01,   7.87880687e-01,   4.89074173e+01,         7.21178881e-01,   5.12612719e+01,   6.54242487e-01,         5.36067547e+01,   5.87562124e-01,   5.59606301e+01,         5.20646946e-01,   5.83061351e+01,   4.53988031e-01,         6.06600314e+01,   3.87094066e-01,   6.30055584e+01,         3.20456605e-01,   6.53594756e+01,   2.53583853e-01,         6.77050244e+01,   1.86967849e-01,   7.00589627e+01,         1.20116306e-01,   7.24045333e+01,   5.35217618e-02,         7.47584926e+01,  -1.33085706e-02,   7.71040850e+01,        -7.98816538e-02,   7.94580653e+01,  -1.46690777e-01,         8.18036794e+01,  -2.13242398e-01,   8.41576809e+01,        -2.80030313e-01,   8.65033165e+01,  -3.46560469e-01,         8.88573391e+01,  -4.13327176e-01,   9.12029964e+01,        -4.79835868e-01,   9.35570401e+01,  -5.46581367e-01,         9.59027190e+01,  -6.13068592e-01,   9.82567838e+01,        -6.79792885e-01,   1.00602484e+02,  -7.46258643e-01,         1.02956570e+02,  -8.12961728e-01,   1.05302292e+02,        -8.79406019e-01,   1.07656399e+02,  -9.46087891e-01,         1.10002186e+02,  -1.01251193e+00])
    #traj = np.reshape(traj, (len(traj)//2, 2))
    font = {'family': 'serif',
            'color': 'darkred',
            'weight': 'normal',
            'size': 16,
            }

    draw_object(new_sim.scenario)
    #plt.plot(traj[:,0], traj[:,1])
    #draw_object(final_state)
    plt.gca().set_aspect('equal')
    plt.savefig(os.getcwd() + "/solution_pic/commonroadUS28.png")
    plt.show()
#visualize_commmonroad()

def veh_model_comp():

    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_xlim(-0.5,1)
    ax3.set_ylim(-.5,1.5)
    for p in [
        patches.Rectangle(
            (0.1, 0.1), 0.4, 0.8,

        )
    ]:
        ax3.add_patch(p)
    c=[]
    m = Vehicle(3, 0.8, 0.3, 0.16)
    pos = ([0.3,0.5], [0.3,0.5])
    orient = [np.pi/2, np.pi/2]
    allign = m.alignment(pos, orient)
    for i in allign:
        ax3.plot(i[:,0],i[:,1])
        for j in range(len(i[:,0])):
            c.append(plt.Circle((i[j,0],i[j,1]), 0.25, color='g', fill=False))
    #plt.plot(pos[:,0],pos[:,1])
    for i in c:
        ax3.add_artist(i)
    #fig3.savefig(os.getcwd() + "/solution_pic/carshape.png")
    fig3.show()


#veh_model_comp()

def vis_reduced():
    dyn,radius = new_sim.dynamic_obstacles_boundary_reduced()
    x  =  np.shape(dyn)
    points = np.zeros((x[0],x[2],x[3]))
    for i in range(len(dyn)):
        points[i]= dyn [i,0,:,:]
    c = []
    fig, ax = plt.subplots(1, 1)
    pointsright, pointsleft = new_sim.generating_driving_corridor
    lower_x, upper_x, lower_y, upper_y = bounds(pointsleft, pointsright)
    ax.set_xlim((lower_x, upper_x))
    ax.set_ylim((lower_y, upper_y))
    draw_object(new_sim.scenario.lanelet_network)
    counter = 0
    for i in points:

        ax.plot(i[:, 0], i[:, 1])
        for j in range(len(i[:,0])):
            c.append(plt.Circle((i[j, 0], i[j, 1]), radius[counter], color='g', fill=False))
        counter += 1
    # plt.plot(pos[:,0],pos[:,1])

    for i in c:
        ax.add_artist(i)
        ax.plot()
    fig.gca().set_aspect('equal')

    #fig.savefig(os.getcwd() + "/solution_pic/newcollision.png",dpi = 300)
    fig.show()


#vis_reduced()

def vis_dynamic():
    dyn = new_sim.dynamic_obstacles_boundary()
    x = np.shape(dyn)
    points = np.zeros((x[0],x[2],x[3]))
    fig, ax = plt.subplots(1, 1)
    pointsright, pointsleft = new_sim.generating_driving_corridor
    lower_x, upper_x, lower_y, upper_y = bounds(pointsleft, pointsright)
    ax.set_xlim((lower_x, upper_x))
    ax.set_ylim((lower_y, upper_y))
    draw_object(new_sim.scenario)
    z= []
    y=[]
    for i in range(x[0]):
        for j in range(x[2]):
            z.append(dyn[i][0][j][0])
            y.append(dyn[i][0][j][1])
            # ax.add_artist(dyn[i][0][j])
            # ax.plot()
    plt.plot(z,y,'ro')
    plt.show()
    fig.gca().set_aspect('equal')
    fig.show()

#vis_dynamic()
