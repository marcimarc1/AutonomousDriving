"""
This module contains the generic Distance class(Abstract)
"""
from abc import ABC, abstractmethod
import numpy as np
from bertha.environment.obstacle import *
from typing import List

class Distance(ABC):
    """
    Abstract Distance class to provide the skeleton and generic behaviour expected
    of a Distance condition.
    """
    def __init__(self):
        """
        Initialize a Distance class
        """
        super().__init__()

    @abstractmethod
    def compute_distance (self, point_a: np.ndarray, point_b: np.ndarray, axis=None):
        """
        compute distance between the two points provided.
        """
        pass
    @abstractmethod
    def compute_distance_static_obstacles (self, trajectory: np.ndarray, obstacles: List[StaticObstacle], axis=None):
        pass
    
    # @abstractmethod
    # def compute_distance_dynamic_obstacle (self, trajectory: np.ndarray, obstacles: List[StaticObstacle], axis=None):
    #     pass

class EuclideanDistance(Distance):
    def compute_distance (self, point_a: np.ndarray, vehicle_point: np.ndarray, radius: float=1.0, axis=None):
        """
        compute distance between the two points provided.
        """
        return np.linalg.norm((point_a - vehicle_point), axis=axis)
    
    def compute_distance_static_obstacle (self, trajectory: np.ndarray, obstacles: StaticObstacleHolder, axis=None):
        """
        compute distance between the two points provided.
        """
        pass
        #return np.linalg.norm((point_a - vehicle_point), axis=axis)


class PsuedoDistance(Distance):
    def compute_distance (self, point_a: np.ndarray, point_b: np.ndarray, axis=None):
        """
        compute distance between the two points provided.
        """
        pass
    def compute_distance_static_obstacles (self, trajectory: np.ndarray, obstacles: StaticObstacleHolder, axis=None):
        dists = []
        nlambdas = []
        for obstacle in obstacles.get_obstacles():
            dist, nlambda, _ = obstacle.compute_distance(trajectory)
            nlambdas.append(nlambda)
            dists.append(dist)
        dists = np.array(dists).T
        dists_indices = np.argmin(np.abs(dists), axis=1)
        nlambdas = np.array(nlambdas)
        nlambdas_n =[]
        dists_n = []
        for i in range(len(dists_indices)):
            nlambdas_n.append(nlambdas[dists_indices[i], i, :])
            dists_n.append(dists[i, dists_indices[i]])
        nlambdas_n = np.array(nlambdas_n)
        dists_n = np.array(dists_n)
        return dists_n, nlambdas_n
