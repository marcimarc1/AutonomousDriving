"""
This module contains the generic Vehicle class
TODO: make this class more generic and may be abstract
"""
class Vehicle:
    """
    Vehicle class to keep track of the vehicle model
    """
    def __init__(self, kappa_max: float, kappa_min: float):
        """
        Initialize an Vehicle class
        :param kappa_max: max steering angle
        :param kappa_min: min steering angle
        """
        self.kappa_max = kappa_max 
        self.kappa_min = kappa_min
        
    def get_kappa_max(self):
        return self.kappa_max
    
    def get_kappa_min(self):
        return self.kappa_min