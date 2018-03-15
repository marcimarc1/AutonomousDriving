import numpy as np
from fvks.scenario.commonroad.file_reader import CommonRoadFileReader

class constraints:

    def __init__(self, filename):
        self.scenario = CommonRoadFileReader(filename).open()

class velocityconsraint(constraints):
    '''
        Calculates  the velocity contraints:
            :parameter
            dot_left: derivative of the compute_distance left to the vehicle at time t
            dot_right: derivative of the compute_distance right to the vehicle at time t
            position: number of the lanelet_id
            velocity: [dot_x(t) , dot_y(t)]

    '''
    def __init__(self, filename):
        self.scenario = CommonRoadFileReader(filename).open()
        self.vdeslanelt = np.array([self.senario.lanelet_network.lanelets.lanelet_id], [self.scenario.lanelets.speed_limit])

    def calculatevdes(self, dot_left, dot_right, position):

        vdes = self.vdeslanelt(position, 2)

        self.vdes = 0.5 * vdes * np.array([0, -1], [1, 0]) @ (dot_left + dot_right)

    def jvel(self,velocity):

        self.j = np.abs(self.vdes - velocity)**2

class offsetconstraint(constraints):
    '''
    calculates the offsetconstraint
    :parameter
    dleft: compute_distance to the left at time t
    dright: compute_distance to the right at time t

    '''
    def joffs(self, dleft, dright):
        self.offset = np.abs(0.5 * (dleft + dright))**2

class smoothnessterms(constraints):
    '''
    calculates the smoothnessterms acc yerk and yaw
    :parameter
    velocity: [dot_x(t), dot_y(t)]
    acc: [ddot_x(t),ddot_y(t)]
    yerk: [dddot_x(t),dddot_y(t)]
    '''
    def jacc(self, acc):
        self.acc = np.abs(acc)**2

    def jyerk(self, yerk):
        self.yerk = np.abs(yerk)**2

    def jyaw(self, velocity,acc):

        phi = np.arctan(velocity[0,:]/velocity[1,:])
        dot_phi = 1/(1+(velocity[0,:]/velocity[1,:])**2) * (acc[0,:]/acc[1,:])
        self.yaw = dot_phi**2

#TODO :  ideas to determine the derivatives of x ... if we get the optimization toolbox and the derivatives we may simulate a simple trajectory
#TODO : Please add here some ideas and todos for the task