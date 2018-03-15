"""
This module contains the generic Vehicle class
TODO: make this class more generic and may be abstract
"""
class EgoVehicle:
    """
    Vehicle class to keep track of the vehicle model
    """
    def __init__(self, kappa_max: float, acc_max: float):
        """
        Initialize an Vehicle class
        :param kappa_max: max steering angle
        :param kappa_min: min steering angle
        """
        self.kappa_max = kappa_max 
        self.acc_max = acc_max