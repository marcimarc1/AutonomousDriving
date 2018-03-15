import numpy as np
from .obstacle import *


class SimulatedCar(Obstacle):

    def __init__(self, trajectory, dt, car_id, # vehicle_class=None
                 length=4.0,
                 width=1.8):
        Obstacle.__init__(self, car_id, ObstacleRole.dynamic, ObstacleType.car)
        self.trajectory = trajectory
        # self.car_id = car_id
        self.dt = dt
        # self.vehicle_class = vehicle_class
        self.length = length
        self.width = width
        self.current_time_idx = self.trajectory.t0
        self.update_state()
        self.active = False
        self.position = None
        self.orientation = None
        self.speed = None

    def update_time(self, time):
        self.current_time_idx = time
        self.update_state()

    def update_state(self):
        tmp = self.trajectory.get_state_at_time(self.current_time_idx)
        if tmp is not None:
            self.position = tmp.position
            self.orientation = tmp.orientation
            try:
                self.speed = tmp.velocity
            except AttributeError:
                adj_state = self.trajectory.get_state_at_time(self.current_time_idx + 1)
                if adj_state is None:
                    adj_state = self.trajectory.get_state_at_time(self.current_time_idx - 1)
                    if adj_state is None:
                        raise Exception()
                self.speed = 1 / self.dt * np.linalg.norm(tmp.position - adj_state.position)
            self.active = True
        else:
            self.active = False

    def state_at_time(self, time):
        print('deprecated: SimulatedCar state_at_time')
        time_idx = time - self.trajectory.t0
        if time_idx >= len(self.trajectory.vertices) or time_idx < 0:
            return None

        return self.trajectory.state_list[time_idx]

        if self.trajectory.velocity is not None:
            speed = self.trajectory.velocity[time_idx]
        else:
            if time_idx < len(self.trajectory.vertices) - 1:
                speed = 1 / self.dt * np.linalg.norm(
                    self.trajectory.vertices[time_idx + 1] -
                    self.trajectory.vertices[time_idx])
            else:
                speed = 1 / self.dt * np.linalg.norm(
                    self.trajectory.vertices[time_idx] -
                    self.trajectory.vertices[time_idx - 1])

        return (self.trajectory.vertices[time_idx],
                self.trajectory.orientation[time_idx],
                speed)

    def create_collision_object(self):
        return fvks.scenario.trajectory.collision_object_from_trajectory(
            self.trajectory, None, None, self.width, self.length)

    def translate_rotate(self, translation, angle):
        self.trajectory.translate_rotate(translation, angle)

    def __str__(self):
        if self.active:
            return (("id: {:4d}  position: {:06.1f}/{:06.1f}  orientation: "
                     "{:04.2f}  speed: {:06.2f}").format(
                self.obstacle_id, self.position[0], self.position[1],
                self.orientation, self.speed))
        else:
            return "not active"

