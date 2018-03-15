"""
This module contains the CostComponents class
"""

from bertha.helpers.helpermethods import grad, psi, normalize, reshaper
import numpy as np
from bertha.helpers.distance import *
from bertha.environment.obstacle import StaticObstacleHolder
from bertha.environment.drivearea import morphingcommonroad

class CostComponents:
    """
    constraints class defines componenets of the bertha cost function
    """
    def __init__(self, distance: Distance, w: np.ndarray, obstacles_left: StaticObstacleHolder,
                 obstacles_right: StaticObstacleHolder, scenario: morphingcommonroad):
        """
        Initialize a CostComponents class
        """
        # TODO: please dont use l_left, l_right and v_des no more, just use 
        # scenario to get lanelet boundaries and lanlet speed limit
        self.distance = distance
        self.w = w
        self.obstacles_left = obstacles_left
        self.obstacles_right = obstacles_right
        self.scenario = scenario
        self.v_des = self.scenario.get_speed_limit()

    def set_w(self, w: np.ndarray):
        self.w = w

    def cost_function(self, x: np.ndarray):
        x = reshaper(x)  # scipy flattens array
        dist_left, n_left = self.distance.compute_distance_static_obstacles(x, self.obstacles_left)
        dist_right, n_right = self.distance.compute_distance_static_obstacles(x, self.obstacles_right)
        _joffs = (self._joffs(x, dist_left, dist_right))
        _jvel =  (self._jvel(x, n_left / np.abs(dist_left)[:, None], n_right / np.abs(dist_right)[:, None]))
        _jacc = normalize(self._jacc(x))
        _jjerk = normalize(self._jjerk(x))
        _jyawr = (self._jyawr(x))
        j = np.array([_joffs, _jvel, _jacc, _jjerk, _jyawr])
        # print(j)
        cost = np.sum(np.dot(self.w.T, j)) #/ np.sum(self.w)
        return cost

    def get_joffs(self):
        joffs = lambda x: self._joffs(x)
        return joffs
    
    def get_jvel(self):
        jvel = lambda x: self._jvel(x)
        return jvel
    
    def get_jacc(self):
        jacc = lambda x: self._jacc(x)
        return jacc
    
    def get_jjerk(self):
        jjerk = lambda x: self._jjerk(x)
        return jjerk
    
    def get_jyawr(self):
        jyawr = lambda x: self._jyawr(x)
        return jyawr


    def _joffs(self, x, dleft, dright):
        return np.power(np.abs(0.5 * (dleft + dright)),2)

    def _calculatevdes(self, x, ddright, ddleft):
        return np.matmul(self.v_des * np.array([[0, -1], [1, 0]], dtype='float64'),
                        0.5 * (np.add(ddleft, ddright).T)).T
    
    def _jvel(self, x, ddleft, ddright):
        d, _, _ = grad(x)
        udes = self._calculatevdes(x, ddleft, ddright)
        return np.linalg.norm(np.subtract(udes, d), axis=1)
    
    def _jacc(self, x):
        d, _, _ = grad(x, 2)
        return np.power(np.abs(np.linalg.norm(d, axis=1)), 2)
    
    def _jjerk(self, x):
        d, _, _ = grad(x, 3)
        return np.power(np.abs(np.linalg.norm(d, axis=1)), 2)

    def _jyawr(self, x):
        d, _, _ = grad(psi(x), nth=1, n=1)
        return np.power(d, 2)
