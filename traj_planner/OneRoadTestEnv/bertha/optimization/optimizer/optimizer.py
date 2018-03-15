"""
This module contains the generic Constraint class(Abstract)
"""
from abc import ABC, abstractmethod
from typing import Callable, Any
from bertha.optimization.optimizer.problem import Problem
from scipy.optimize import minimize
import numpy as np

class Optimizer(ABC):
    """
    Abstract Optimizer class to provide the skeleton and generic behaviour expected
    from every Optimizer
    """
    def __init__(self, optimization_problem: Problem):
        """
        Initialize an Optimizer class
        :param obj: Callable Objective function that takes only one argument
        """
        self.problem = optimization_problem
        super().__init__()

    @abstractmethod
    def optimize(self):
        """
        Get the Optimizer specific to a library or optimizer.
        for example scipy SLSQP requires : { type: 'eq', func: @callable}
        """
        pass


class ScipyOptimizer(Optimizer):
    """
    Optimizer class to provide the skeleton and generic behaviour expected
    from every Optimizer
    """
    def __init__(self, optimization_problem: Problem, options):
        """
        Initialize an Optimizer class
        :param obj: Callable Objective function that takes only one argument
        """
        self.options = options
        super().__init__(optimization_problem)

    def set_options(self, options):
       self.options = options

    def optimize(self, initial_guess: np.ndarray) -> Any:
        """
        Optimize specific to a library or optimizer.
        for example scipy
        param: initial_guess: the initial guess for the optimizer to start optimization.
        """
        return minimize(self.problem.objective, initial_guess,constraints=self.problem.constraints, method='SLSQP', options=self.options)