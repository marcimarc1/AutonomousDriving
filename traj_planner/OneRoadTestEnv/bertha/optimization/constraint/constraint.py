"""
This module contains the generic Constraint class(Abstract)
"""
from abc import ABC, abstractmethod
from typing import Callable, Any

class Constraint(ABC):
    """
    Abstract constraints class to provide the skeleton and generic behaviour expected
    from every constraint
    """
    def __init__(self, func: Callable[[Any], Any], typ: str = "eq"):
        """
        Initiylize a Constraint class
        :param func: Callable constraint function that takes only one argument
        """
        self.func = func
        self.type =typ
        super().__init__()

    @abstractmethod
    def get_constraint(self):
        """
        Get the constraint specific to a library or optimizer.
        for example scipy SLSQP requires : { type: 'eq', func: @callable}
        """
        pass


class ScipyConstraint(Constraint):
    def get_constraint(self):
        return {
            "type": self.type,
            "fun": self.func
        }
