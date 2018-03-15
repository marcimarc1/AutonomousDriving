"""
This module contains the generic Constraint class(Abstract)
"""
from abc import ABC, abstractmethod
from typing import Callable, Any
import numpy as np

from bertha.environment.drivearea import morphingcommonroad

class Boundary(ABC):
    """
    Abstract Boundary class to provide the skeleton and generic behaviour expected
    of a boundary condition.
    """
    def __init__(self, scenario: morphingcommonroad, length: int):
        """
        Initialize a Boundary class
        :param scenario: instance of morphingcommonroad which is a wrapper arround
        Commonroad that provides us helper methods to work with Commonroad
        :param length int: provides the length of the trajectory.
        """
        self.scenario = scenario
        self.length = length
        self.boundaries = None
        super().__init__()

    @abstractmethod
    def set_boundaries(self):
        """
        Set the boundaries sperific to a library or optimizer
        for example scipy SLSQP requires : (lower,upper) for every control point.
        """
        pass

    @abstractmethod
    def get_boundaries(self):
        """
        Get the boundaries specific to a library or optimizer.
        for example scipy SLSQP requires : (lower,upper) for every control point.
        """
        pass
