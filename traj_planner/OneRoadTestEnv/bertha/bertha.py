"""
This module contains the Bertha implementation class
"""
class Berhta:
    """
    Implementation of the whole bertha optimization problem we need to setup 
    constraints, objective, boundaries, environment.
    in essence this Class unites all.
    """
    def __init__(self):
        """
        Initialize an Optimizer class
        :param obj: Callable Objective function that takes only one argument
        """
        super().__init__()