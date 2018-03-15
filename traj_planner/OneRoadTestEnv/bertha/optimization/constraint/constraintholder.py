"""
This module contains the ConstraintHolder class
"""
from bertha.optimization.constraint.constraint import Constraint
from typing import List
class ConstraintHolder:
    """
    constraints class holds all the constraints and returns constraints in a
    convenient fashion.
    """
    def __init__(self):
        """
        Initialize a ConstraintHolder class
        """
        self.constraints = []
        self.resolved_constraints = []

    def add_constraint(self, constraint: Constraint, active: bool=True) -> List:
        """
        add a constraint prodided to the constraints array
        :param constraint: An instance of type Constraint
        :param active: whether to include this constraint or not
        :return updated constraints array
        """
        con = {'con': constraint, 'active': active}
        self.constraints.append(con)
        if(active):
            self.resolved_constraints.append(constraint.get_constraint())
        return self.constraints

    def get_constraint_array(self) -> List:
        """
        get all active constraints in a compliant format.
        :return active constraint array
        """
        return self.resolved_constraints