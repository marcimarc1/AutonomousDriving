"""
Computes initial trajectory using RRT
"""

import os
import matplotlib.pyplot as plt
from fvks.scenario.commonroad.file_reader import CommonRoadFileReader
from fvks.visualization.draw_dispatch import draw_object
from fvks.planning.vehicle_models import VehicleModelFactory
from rrt import StatePropagator, StateSamplerPosition, RRT
import pickle

# load example scenario with planning task
scenario, planning_task = CommonRoadFileReader(os.getcwd() + '/OneRoad.xml').open()

# maximum number of samples
n_max_sampling = 700
# create collision checker from scenario
collision_checker = scenario.create_collision_checker()

# define state space for sampling
state_space = StateSamplerPosition([0, 100], [0, 5])
# select vehicle model and the parameters
vehicle_model = VehicleModelFactory.create('KS', 1)
# state propagator steers vehicle from a start state
# to a goal state. Each propagated trajectory has
# duration of duration_trajectory*scenario.dt
duration_trajectory = 10
state_propagator = StatePropagator(vehicle_model, duration_trajectory, scenario.dt)

plt.figure(figsize=(25, 25))
g_tr = 0
# solve each planning problem sequentially with RRT
for i, p in enumerate(planning_task.planning_problems):
    print('Solving planning problem ', p.planning_problem_id)

    # initialize RRT and plan trajectory
    rrt = RRT(state_space, state_propagator, p, scenario.dt)
    rrt.plan(n_max_sampling, collision_checker, scenario)

    # plot tree of RRT
    plt.subplot(1, len(planning_task.planning_problems), i + 1)
    draw_object(scenario)
    rrt.draw()
    plt.gca().set_aspect('equal')

    # avoid collisions between ego vehicles, therefore add found
    # trajectory to the collision checker
    if p.has_solution():
        g_tr = rrt._get_trajectory(rrt.node_list[-1])
        trajectory = [{
                'position': s.position, 
                'orientation': s.orientation, 
                'steeringAngle': s.steeringAngle, 
                'time': s.time
            } for s in g_tr.state_list]
        pickle_out = open('trajectory.p', 'wb')
        pickle.dump(trajectory, pickle_out)
        pickle_out.close()
        collision_checker.add_collision_object(p.car.create_collision_object())
plt.show()