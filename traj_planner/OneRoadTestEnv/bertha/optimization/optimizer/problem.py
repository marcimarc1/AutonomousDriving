"""
This module contains the Optimization Problem definition
"""
from typing import Callable, Any
from bertha.optimization.constraint.constraintholder import ConstraintHolder
class Problem:
    """
    Problem class to provide the our optimizer with our optimization problem.
    """
    def __init__(self, objective: Callable[[Any], Any], constraints: ConstraintHolder, boundaries: Any):
        """
        Initialize an Optimizer class
        :param obj: Callable Objective function that takes only one argument
        """
        self.objective = objective
        self.constraints = constraints.get_constraint_array()
        self.boundaries = boundaries

    def is_constrainted(self):
        return not(not(self.constraints))

    def is_bounded(self):
        return not(not(self.boundaries))
