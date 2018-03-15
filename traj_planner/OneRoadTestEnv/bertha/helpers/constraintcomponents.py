"""
This module contains the ConstraintComponents class
"""

from bertha.helpers.helpermethods import euclid_dist, grad, kappa, reshaper
from bertha.environment.collision_model_bertha import Vehicle
from bertha.helpers.distance import StaticObstacleHolder
import numpy as np
from bertha.environment.drivearea import morphingcommonroad, bounds, goal_bounds  # , constraint_interpolation
from bertha.helpers.distance import Distance
import numexpr as ne
class ConstraintComponents:
    """
    constraints class defines componenets of the bertha cost function
    """
    def __init__(self, scenario: morphingcommonroad, vehicle: Vehicle,
                 obstacles_left: StaticObstacleHolder, obstacles_right: StaticObstacleHolder,
                 distance: Distance, dynamic_bounds: np.ndarray,
                 init_state: np.ndarray, goal: np.ndarray):
        """
        Initialize a CostComponents class
        :param kmax float: maximum steering angle
        """
        self.scenario = scenario
        self.kmax = vehicle.kmax
        self.vmax = scenario.get_speed_limit()
        self.amax = vehicle.amax
        self.init_vel = vehicle.init_vel
        self.obstacles_left = obstacles_left
        self.obstacles_right = obstacles_right
        self.distance = distance
        self.timesteps = self.scenario.get_time_by_dynamic_obstacles()
        self.dynamic_bounds = dynamic_bounds
        self.init_state = init_state
        self.goal = goal
        print(self.init_vel)

    def kappa_constraint(self, x):
        x = reshaper(x)
        # print(x.shape)
        # −κmax ≤ κ(t) ≤ κmax.
        # TODO: get κmax from vehicle model.
        kap = kappa(x)
        kap = self.kmax - kap
        return kap

    def kappa_constraint_neg(self, x):
        x = reshaper(x)
        # print(x.shape)
        # −κmax ≤ κ(t) ≤ κmax.
        # TODO: get κmax from vehicle model.
        kap = kappa(x)
        kap = self.kmax + kap
        return kap

    def acc_constraint(self,  x):
        x = reshaper(x)
        # TODO: get acc from scenario -> there is no constraint in the scenario -> see vehicle model
        acc, _, _ = grad(x, nth=2, n=2)
        acc = np.linalg.norm(acc, axis=1)
        # print(acc.shape)
        acc_con = self.amax**2 - acc**2
        return acc_con


    def drive_between_corridor_left(self, x):
        x = reshaper(x)
        dist_left, _ = self.distance.compute_distance_static_obstacles(x, self.obstacles_left)
        return dist_left

    def drive_between_corridor_right(self, x):
        x = reshaper(x)
        dist_right, _ = self.distance.compute_distance_static_obstacles(x, self.obstacles_right)
        return -1.*dist_right


    def vel_constraint(self, x):
        x = reshaper(x)
        d, _, _ = grad(x)
        vel = np.linalg.norm(d)
        return (self.vmax - vel)


    def collision_constraint(self, x):
        """
        x = (10,2) => (10*10,2) => (10*10*3,2) => (2,10*10*3,2)
        d_b = (2,10,3,2) => (2,30,2) => (2,10*30,2)

        """
        x = reshaper(x)
        x_n = np.repeat(x, len(x), axis=0)
        d_b_shape = self.dynamic_bounds.shape
        x_n = np.repeat(x_n, self.dynamic_bounds.shape[2], axis=0)
        x_n = np.reshape(np.tile(x_n, (len(self.dynamic_bounds), 1)),
                         (len(self.dynamic_bounds), (len(x)**2)*d_b_shape[2], 2))
        d_b = np.reshape(self.dynamic_bounds,
                         (len(self.dynamic_bounds), d_b_shape[1]*d_b_shape[2], 2))
        d_b = np.tile(d_b, (1,len(x),1))
        dists = np.linalg.norm(ne.evaluate('d_b - x_n'), axis=2)  - 1.5 - 1.5

        # for i in np.arange(self.timesteps):
        #     for d_obs in np.array(self.dynamic_bounds):
        #         n_d_obs = np.reshape(d_obs, (d_obs.shape[0] * d_obs.shape[1], 2))
        #         n_x = np.repeat([x[i]], self.timesteps * 4, axis=0)
        #         dist = euclid_dist(n_x, n_d_obs) - 1.
        #         dists.append(dist.flatten())
        return dists.flatten()

    def ini_vel_constraint(self, x):
        x = reshaper(x)
        d, _, _ = grad(x)
        grad_mag = np.linalg.norm(d, axis=1)
        return self.init_vel - grad_mag[1]

    def ini_state_constraint(self, x):
        x = reshaper(x)
        return np.abs(self.init_state - x[0])

    def dist_to_goal(self, x):
        x = reshaper(x)
        #print(-1.*euclid_dist(np.array([1.10e+02,  0.11742588e+00]), x[-1], axis=0))
        return 2 + -1.*euclid_dist(np.array(self.goal), x[-1], axis=0)