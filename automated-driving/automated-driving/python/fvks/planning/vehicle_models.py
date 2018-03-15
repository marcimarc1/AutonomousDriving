import abc
import numpy as np
from scipy.integrate import odeint, ode

import init_KS, init_MB, init_ST
import vehicleDynamics_KS, vehicleDynamics_MB, vehicleDynamics_ST
import parameters_vehicle1, parameters_vehicle2, parameters_vehicle3

from fvks.scenario.trajectory import StateTupleFactory
from fvks.scenario.trajectory import Trajectory, collision_object_from_trajectory


class VehicleModelFactory:
    @classmethod
    def create(cls, model_id, vehicle_id):
        if vehicle_id == 1:
            parameter = parameters_vehicle1.parameters_vehicle1()
        elif vehicle_id == 2:
            parameter = parameters_vehicle2.parameters_vehicle2()
        elif vehicle_id == 3:
            parameter = parameters_vehicle3.parameters_vehicle3()
        else:
            raise ValueError("Only vehicle ids 1, 2 or 3 are supported.")

        if model_id == 'KS':
            return KinematicSingleTrackModel(parameter)
        elif model_id == 'ST':
            return SingleTrackModel(parameter)
        elif model_id == 'MB':
            return MultiBodyModel(parameter)
        else:
            raise ValueError("Only vehicle models KS, ST and MB are supported.")


class VehicleModel(metaclass=abc.ABCMeta):
    @abc.abstractclassmethod
    def initialize(self, state):
        pass

    @abc.abstractclassmethod
    def forward_simulation(self, x0, t0, u, dt):
        pass

    def integrate(self, x0, t0, u, dt, method='dopri5', nsteps=50000):
        x = [x0]
        for k in range(len(u)):
            solver = ode(self.dynamics).set_integrator(method, nsteps=nsteps)\
                                       .set_initial_value(x[-1], k * dt)\
                                       .set_f_params(u[k])
            solver.integrate((k + 1) * dt)
            # if not solver.successful():
            #     raise Exception()
            x.append(solver.y)
        return x

    @classmethod
    def constraints_fulfilled(cls, trajectory):
        fulfilled = True
        for state in trajectory.state_list:
            if state.velocity < 0 or state.velocity > 50.8:
                print('Warning: velocity bounds are exceeded.')
                fulfilled = False
            if state.steeringAngle < -0.90 or state.steeringAngle > 0.90:
                print('Warning: steering angle bounds are exceeded.')
                fulfilled = False
        return fulfilled


class KinematicSingleTrackModel(VehicleModel):
    state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position,
                                                       StateTupleFactory.steeringAngle,
                                                       StateTupleFactory.velocity,
                                                       StateTupleFactory.orientation,
                                                       StateTupleFactory.time)

    def __init__(self, parameter):
        self.parameter = parameter

    def dynamics(self, t, x, u):
        return vehicleDynamics_KS.vehicleDynamics_KS(x, u, self.parameter)

    def initialize(self, state):
        x_start = [state.position[0], state.position[1], state.velocity,
                   state.orientation, state.yawRate, state.slipAngle]
        x0_ = init_KS.init_KS(x_start, self.parameter)
        x0 = self.convert_state_to_fvks(x0_, state.time)
        return x0

    def forward_simulation(self, x0, t0, u, dt):
        x0_ = self.convert_to_commonroad(x0)
        x_ = self.integrate(x0_, t0, u, dt, method='dopri5', nsteps=5000)
        t = range(t0, t0 + len(u) + 1)
        x = self.convert_to_fvks(x_, t)

        trajectory = Trajectory(t[0], x, KinematicSingleTrackModel.state_tuple)
        collision_object = collision_object_from_trajectory(trajectory,
                                                            CAR_WIDTH=self.parameter.w,
                                                            CAR_LENGTH=self.parameter.l)
        constraints_fulfilled = self.constraints_fulfilled(trajectory)
        return trajectory, collision_object, constraints_fulfilled

    @classmethod
    def convert_to_commonroad(cls, state):
        x = np.empty(5)
        x[0] = state.position[0]
        x[1] = state.position[1]
        x[2] = state.steeringAngle
        x[3] = state.velocity
        x[4] = state.orientation
        return x

    @classmethod
    def convert_state_to_fvks(cls, x, t):
        return KinematicSingleTrackModel.state_tuple(np.array([x[0], x[1]]), x[2], x[3], x[4], t)

    @classmethod
    def convert_to_fvks(cls, x, t):
        state_list = list()
        for i, x_ in enumerate(x):
            state_list.append(cls.convert_state_to_fvks(x_, t[i]))
        return state_list


class SingleTrackModel(VehicleModel):
    state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position,
                                                       StateTupleFactory.steeringAngle,
                                                       StateTupleFactory.velocity,
                                                       StateTupleFactory.orientation,
                                                       StateTupleFactory.yawRate,
                                                       StateTupleFactory.slipAngle,
                                                       StateTupleFactory.time)

    def __init__(self, parameter):
        self.parameter = parameter

    def dynamics(self, t, x, u):
        return vehicleDynamics_ST.vehicleDynamics_ST(x, u, self.parameter)

    def initialize(self, state):
        x_start = [state.position[0], state.position[1], state.velocity,
                   state.orientation, state.yawRate, state.slipAngle]

        x0_ = init_ST.init_ST(x_start, self.parameter)
        x0 = self.convert_state_to_fvks(x0_, state.time)
        return x0

    def forward_simulation(self, x0, t0, u, dt):
        x0_ = self.convert_to_commonroad(x0)
        x_ = self.integrate(x0_, t0, u, dt, method='dopri5', nsteps=5000)
        t = range(t0, t0 + len(u) + 1)
        x = self.convert_to_fvks(x_, t)
        trajectory = Trajectory(t[0], x, SingleTrackModel.state_tuple)
        collision_object = collision_object_from_trajectory(trajectory,
                                                            CAR_WIDTH=self.parameter.w,
                                                            CAR_LENGTH=self.parameter.l)
        constraints_fulfilled = self.constraints_fulfilled(trajectory)
        return trajectory, collision_object, constraints_fulfilled

    @classmethod
    def convert_to_commonroad(cls, state):
        x = np.empty(7)
        x[0] = state.position[0]
        x[1] = state.position[1]
        x[2] = state.steeringAngle
        x[3] = state.velocity
        x[4] = state.orientation
        x[5] = state.yawRate
        x[6] = state.slipAngle
        return x

    @classmethod
    def convert_state_to_fvks(cls, x, t):
        return SingleTrackModel.state_tuple(np.array([x[0], x[1]]), x[2], x[3], x[4], x[5], x[6], t)

    @classmethod
    def convert_to_fvks(cls, x, t):
        state_list = list()
        for i, x_ in enumerate(x):
            state_list.append(cls.convert_state_to_fvks(x_, t[i]))
        return state_list


class MultiBodyModel(VehicleModel):
    state_tuple = StateTupleFactory.create_state_tuple(StateTupleFactory.position,
                                                       StateTupleFactory.steeringAngle,
                                                       StateTupleFactory.velocity,
                                                       StateTupleFactory.orientation,
                                                       StateTupleFactory.yawRate,
                                                       StateTupleFactory.rollAngle,
                                                       StateTupleFactory.rollRate,
                                                       StateTupleFactory.pitchAngle,
                                                       StateTupleFactory.pitchRate,
                                                       StateTupleFactory.yVelocity,
                                                       StateTupleFactory.zPosition,
                                                       StateTupleFactory.zVelocity,
                                                       StateTupleFactory.rollAngleFront,
                                                       StateTupleFactory.rollRateFront,
                                                       StateTupleFactory.yVelocityFront,
                                                       StateTupleFactory.zPositionFront,
                                                       StateTupleFactory.zVelocityFront,
                                                       StateTupleFactory.rollAngleRear,
                                                       StateTupleFactory.rollRateRear,
                                                       StateTupleFactory.yVelocityRear,
                                                       StateTupleFactory.zPositionRear,
                                                       StateTupleFactory.zVelocityRear,
                                                       StateTupleFactory.leftFrontWheelAngularSpeed,
                                                       StateTupleFactory.rightFrontWheelAngularSpeed,
                                                       StateTupleFactory.leftRearWheelAngularSpeed,
                                                       StateTupleFactory.rightRearWheelAngularSpeed,
                                                       StateTupleFactory.deltaYf,
                                                       StateTupleFactory.deltaYr,
                                                       StateTupleFactory.time)

    def __init__(self, parameter):
        self.parameter = parameter

    def dynamics(self, t, x, u):
        return vehicleDynamics_MB.vehicleDynamics_MB(x, u, self.parameter)

    def initialize(self, state):
        x_start = [state.position[0], state.position[1], state.velocity,
                   state.orientation, state.yawRate, state.slipAngle]

        x0_ = init_MB.init_MB(x_start, self.parameter)
        x0 = self.convert_state_to_fvks(x0_, state.time)
        return x0

    def forward_simulation(self, x0, t0, u, dt):
        x0_ = self.convert_to_commonroad(x0)
        x_ = self.integrate(x0_, t0, u, dt, method='dopri5', nsteps=5000)
        t = range(t0, t0 + len(u) + 1)
        x = self.convert_to_fvks(x_, t)
        trajectory = Trajectory(t[0], x, MultiBodyModel.state_tuple)
        collision_object = collision_object_from_trajectory(trajectory,
                                                            CAR_WIDTH=self.parameter.w,
                                                            CAR_LENGTH=self.parameter.l)
        constraints_fulfilled = self.constraints_fulfilled(trajectory)
        return trajectory, collision_object, constraints_fulfilled

    @classmethod
    def convert_to_commonroad(cls, state):
        x = np.empty(29)
        x[0] = state.position[0]
        x[1] = state.position[1]
        x[2] = state.steeringAngle
        x[3] = state.velocity
        x[4] = state.orientation
        x[5] = state.yawRate

        x[6] = state.rollAngle
        x[7] = state.rollRate
        x[8] = state.pitchAngle
        x[9] = state.pitchRate
        x[10] = state.yVelocity
        x[11] = state.zPosition
        x[12] = state.zVelocity

        x[13] = state.rollAngleFront
        x[14] = state.rollRateFront
        x[15] = state.yVelocityFront
        x[16] = state.zPositionFront
        x[17] = state.zVelocityFront

        x[18] = state.rollAngleRear
        x[19] = state.rollRateRear
        x[20] = state.yVelocityRear
        x[21] = state.zPositionRear
        x[22] = state.zVelocityRear

        x[23] = state.leftFrontWheelAngularSpeed
        x[24] = state.rightFrontWheelAngularSpeed
        x[25] = state.leftRearWheelAngularSpeed
        x[26] = state.rightRearWheelAngularSpeed

        x[27] = state.deltaYf
        x[28] = state.deltaYr
        return x

    @classmethod
    def convert_state_to_fvks(cls, x, t):
        return MultiBodyModel.state_tuple(
                np.array([x[0], x[1]]), x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9],
                x[10], x[11], x[12], x[13], x[14], x[15], x[16], x[17],
                x[18], x[19], x[20], x[21], x[22], x[23], x[24], x[25],
                x[26], x[27], x[28], t)

    @classmethod
    def convert_to_fvks(cls, x, t):
        state_list = list()
        for i, x_ in enumerate(x):
            state_list.append(cls.convert_state_to_fvks(x_, t[i]))
        return state_list