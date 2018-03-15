import os
import matplotlib.pyplot as plt
import numpy as np
import math
from bertha.environment.drivearea import morphingcommonroad, bounds, goal_bounds  # , constraint_interpolation
from  bertha.environment.collision_model_bertha import Vehicle
from bertha.environment.obstacle import StaticObstacleHolder
from bertha.helpers.distance import *
from bertha.helpers.costcomponents import CostComponents
from bertha.helpers.constraintcomponents import ConstraintComponents
from bertha.helpers.helpermethods import reshaper
from bertha.optimization.constraint.constraintholder import ConstraintHolder
from bertha.optimization.constraint.constraint import ScipyConstraint
from bertha.optimization.optimizer.optimizer import ScipyOptimizer
from bertha.optimization.optimizer.problem import Problem
from fvks.visualization.draw_dispatch import draw_object

class OneRoad:
    def __init__(self, scenario: str='US101/NGSIM_US101_0.xml'):
        cwd = os.getcwd() + '/scenarios/NGSIM/' + scenario
        # cwd = cwd - "OneRoadTestEnv"
        self.cwd = cwd.replace('OneRoadTestEnv/', '')  # sub("OneRoadTestEnv",'')
        self.setup_everything()

    def setup_everything(self):
        self.setup_commonroad()
        self.setup_environment()
        self.setup_vehicle()
        self.setup_inital_guess()
        self.setup_cost_components()
        self.setup_constraints()
        self.setup_optimization()

    def setup_inital_guess(self):
        from fvks.scenario import Lanelet
        if(not(self.final_state) or isinstance(self.final_state[0], Lanelet)):
            self.final_state = np.array([100, -1]) # TODO: replace with a mechanism
        self.xs = np.linspace(-31., self.final_state[0], self.timesteps)
        self.ys = np.linspace(3., self.final_state[1], self.timesteps)
        self.states = np.array(list(zip(self.xs, self.ys)))
        self.new_tr = self.states

    def setup_commonroad(self):
        self.new_sim = morphingcommonroad(self.cwd)
        self.planning_task = self.new_sim.get_planning_task()
        self.dynamic_bounds = self.new_sim.dynamic_obstacles_boundary_reduced()
        self.dt = self.new_sim.scenario.dt
        self.timesteps = self.new_sim.get_time_by_dynamic_obstacles()
        self.pointsright, self.pointsleft = self.new_sim.generating_driving_corridor

    def setup_environment(self, goal: np.ndarray=None):
        self.distance = PsuedoDistance()
        self.final_state1 = self.planning_task.planning_problems[0].goal
        self.final_state = self.final_state1.state_list[0][0]
        self.initial_state = self.planning_task.planning_problems[0].initial_state
        self.time_of_final_state = self.final_state1.state_list[0].time
        self.obstacles_left = StaticObstacleHolder(self.pointsleft, self.dt)
        self.obstacles_right = StaticObstacleHolder(self.pointsright, self.dt, True)
        self.v_des = self.new_sim.get_speed_limit()
        self.init_vel = self.initial_state.velocity
        self.initial_position = self.initial_state.position


    def setup_vehicle(self):
        kmax = np.pi / 1000.
        amax = 5.
        self.vmax = self.new_sim.get_speed_limit()
        car_length = 4
        car_width = 2
        r_veh = math.sqrt(2)
        self.vehicle_model = Vehicle(4, car_length, car_width, r_veh, kmax, amax, self.init_vel)

    def setup_constraints(self):
        self.constraints = ConstraintComponents(self.new_sim, self.vehicle_model,
                                                self.obstacles_left, self.obstacles_right,
                                                self.distance, self.dynamic_bounds,
                                                self.initial_position, self.final_state)
        self.constraintsholder = ConstraintHolder()
        self.constraint = ScipyConstraint
        kappa_constraint = self.constraint(self.constraints.kappa_constraint, typ="ineq")
        kappa_constraint_neg = self.constraint(self.constraints.kappa_constraint_neg, typ="ineq")
        acc_constraint = self.constraint(self.constraints.acc_constraint, typ="ineq")
        drive_between_corridor_left = self.constraint(self.constraints.drive_between_corridor_left, typ="ineq")
        drive_between_corridor_right = self.constraint(self.constraints.drive_between_corridor_right, typ="ineq")
        vel_constraint = self.constraint(self.constraints.vel_constraint, typ="ineq")
        collision_constraint = self.constraint(self.constraints.collision_constraint, typ="ineq")
        ini_state_constraint = self.constraint(self.constraints.ini_state_constraint, typ="eq")
        ini_velocity_constraint = self.constraint(self.constraints.ini_vel_constraint, typ="eq")
        dist_to_goal = self.constraint(self.constraints.dist_to_goal, typ="ineq")
        self.constraintsholder.add_constraint(ini_state_constraint, active=True)
        self.constraintsholder.add_constraint(ini_velocity_constraint, active=True)
        self.constraintsholder.add_constraint(kappa_constraint, active=True)
        self.constraintsholder.add_constraint(kappa_constraint_neg, active=True)
        self.constraintsholder.add_constraint(acc_constraint, active=True)
        self.constraintsholder.add_constraint(drive_between_corridor_left, active=True)
        self.constraintsholder.add_constraint(drive_between_corridor_right, active=True)
        self.constraintsholder.add_constraint(vel_constraint, active=True)
        self.constraintsholder.add_constraint(collision_constraint, active=False)
        self.constraintsholder.add_constraint(dist_to_goal, active=True)


    def setup_cost_components(self):
        self.w = np.array([9., 0.1, 0.1, 1., 0.1])
        self.cost_component = CostComponents(self.distance, self.w, self.obstacles_left,
                                             self.obstacles_right, self.new_sim)
        self.cost_function = self.cost_component.cost_function

    def setup_optimization(self):
        self.problem = Problem(self.cost_function, self.constraintsholder, None)
        self.optimization = ScipyOptimizer
        self.optimizer = self.optimization(self.problem,None)

    def run_optmization(self, init_guess, w, options):
        self.optimizer.set_options(options)
        self.w = w
        # self.cost_component.set_w(w)
        return self.optimizer.optimize(init_guess)


# bertha = OneRoad()
# sol = bertha.run_optmization()
# print(len(sol.x))
#
# xf = reshaper(sol.x)
#
# plt.figure()
# font = {'family': 'serif',
#         'color': 'darkred',
#         'weight': 'normal',
#         'size': 16,
#         }
# w_new = np.array2string(bertha.w)
# plt.title('Weights: ' + w_new, fontdict=font)
# x0_c = reshaper(bertha.new_tr)
# plt.plot(x0_c[:, 0], x0_c[:, 1], '-x')
# plt.plot(xf[:, 0], xf[:, 1], '-o')
# plt.plot(bertha.pointsleft[:, 0], bertha.pointsleft[:,1], '--')
# plt.plot(bertha.pointsright[:, 0], bertha.pointsright[:,1], '--')
# draw_object(bertha.new_sim.scenario)
# draw_object(bertha.final_state)
# plt.gca()
# # plt.savefig(os.getcwd() + "/solution_pic/" + name + ".png")
# plt.show()