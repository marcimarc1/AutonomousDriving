import numpy as np
from collections import namedtuple

import pyfvks
import fvks.geometry.polyline
import fvks.geometry.transform

from .scenario_exception import ScenarioError


def is_exact(obj):
    if type(obj)==float or type(obj) == np.float64:
        return True
    else:
        return False


class StateTupleFactory:

    position = 'position'  # x- and y-position in a global coordinate system
    orientation = 'orientation'  # yaw angle
    velocity = 'velocity'  # velocity in local x-direction
    steeringAngle = 'steeringAngle'  # steering angle of front wheels
    yawRate = 'yawRate'  # yaw rate
    slipAngle = 'slipAngle'  # slip angle at vehicle center

    rollAngle = 'rollAngle'  # roll angle
    rollRate = 'rollRate'  # roll rate
    pitchAngle = 'pitchAngle'  # pitch angle
    pitchRate = 'pitchRate'  # pitch rate
    yVelocity = 'yVelocity'  # velocity in local y-direction
    zPosition = 'zPosition'  # z-position
    zVelocity = 'zVelocity'  # velocity in z-direction

    rollAngleFront = 'rollAngleFront'  # roll angle front
    rollRateFront = 'rollRateFront'  # roll rate front
    yVelocityFront = 'yVelocityFront'  # velocity in y-direction front
    zPositionFront = 'zPositionFront'  # z-position front
    zVelocityFront = 'zVelocityFront'  # velocity in z-direction front

    rollAngleRear = 'rollAngleRear'  # roll angle rear
    rollRateRear = 'rollRateRear'  # roll rate rear
    yVelocityRear = 'yVelocityRear'  # velocity in y-direction rear
    zPositionRear = 'zPositionRear'  # z-position rear
    zVelocityRear = 'zVelocityRear'  # velocity in z-direction rear

    leftFrontWheelAngularSpeed = 'leftFrontWheelAngularSpeed'  # left front wheel angular speed
    rightFrontWheelAngularSpeed = 'rightFrontWheelAngularSpeed'  # right front wheel angular speed
    leftRearWheelAngularSpeed = 'leftRearWheelAngularSpeed'  # left rear wheel angular speed
    rightRearWheelAngularSpeed = 'rightRearWheelAngularSpeed'  # right rear wheel angular speed

    deltaYf = 'deltaYf'  # delta_y_f
    deltaYr = 'deltaYr'  # delta_y_r

    acceleration = 'acceleration'
    time = 'time'

    allowed_fields = (position, orientation, velocity, steeringAngle, yawRate, slipAngle,
                      rollAngle, rollRate, pitchAngle, pitchRate, yVelocity, zPosition, zVelocity,
                      rollAngleFront, rollRateFront, yVelocityFront, zPositionFront, zVelocityFront,
                      rollAngleRear, rollRateRear, yVelocityRear, zPositionRear, zVelocityRear,
                      leftFrontWheelAngularSpeed, rightFrontWheelAngularSpeed,
                      leftRearWheelAngularSpeed, rightRearWheelAngularSpeed,
                      deltaYf, deltaYr, acceleration, time)

    @classmethod
    def create_state_tuple(cls, *args):
        for a in args:
            if a not in cls.allowed_fields:
                raise Exception()
        nt = namedtuple('State', args)
        return nt


class Trajectory:

    #def __init__(self, vertices, t0, orientation, velocity):
    #    if len(orientation) != len(vertices):
    #        raise ScenarioError
    #    if velocity is not None and len(vertices) != len(velocity):
    #        raise ScenarioError
    #    self.vertices = vertices
    #    self.t0 = t0
    #    self.orientation = orientation
    #    self.velocity = velocity
    def __init__(self, t0, state_list, state_tuple):
        self.t0 = t0
        self.state_list = state_list
        self.state_tuple = state_tuple

    def get_state_at_time(self, time_idx):
        if not (self.t0 <= time_idx < self.t0 + len(self.state_list)):
            return None
        else:
            return self.state_list[time_idx-self.t0]

    def get_final_state(self):
        return self.state_list[-1]

    @classmethod
    def _create_from_unpacked_states(cls, vertices, t0, orientation,
                                     velocity=None):
        if len(orientation) != len(vertices):
            raise ScenarioError
        if velocity is not None and len(vertices) != len(velocity):
            raise ScenarioError

        list_of_state_fields = [StateTupleFactory.position,
                                StateTupleFactory.orientation]
        if velocity is not None:
            list_of_state_fields.append(StateTupleFactory.velocity)

        list_of_state_fields.append(StateTupleFactory.time)
        state_tuple = StateTupleFactory.create_state_tuple(
            *tuple(list_of_state_fields))

        state_list = list()
        for state_idx in range(len(vertices)):
            if velocity is None:
                state_list.append(state_tuple(vertices[state_idx],
                                              orientation[state_idx],
                                              t0 + state_idx))
            else:
                state_list.append(state_tuple(vertices[state_idx],
                                              orientation[state_idx],
                                              velocity[state_idx],
                                              t0 + state_idx))
        return cls(t0, state_list, state_tuple)

    def create_collision_object(self):
        # Used only in the RRT for the reachabilty comparision?
        tvco = pyfvks.collision_TimeVariantCollisionObject(self.t0)
        tmp = pyfvks.collision_RectOBB(2, 2, 0)
        for i in range(0, len(self.vertices)-1):
            shapegroup = pyfvks.collision_ShapeGroup()
            tmp = pyfvks.collision_Circle(0.9, self.vertices[i][0],
                                          self.vertices[i][1])
            shapegroup.addShape(tmp)
            tmp = pyfvks.collision_Circle(0.9, self.vertices[i+1][0],
                                          self.vertices[i+1][1])
            shapegroup.addShape(tmp)
            ang = np.arctan2(self.vertices[i+1][1]-self.vertices[i][1],
                             self.vertices[i+1][0] - self.vertices[i][0])
            center = 0.5*(self.vertices[i]+self.vertices[i+1])
            length = np.linalg.norm(self.vertices[i+1]-self.vertices[i])
            obb = pyfvks.collision_RectOBB(length/2.0, 0.9, ang)
            obb.set_center(center[0], center[1])
            shapegroup.addShape(obb)
            tvco.appendObstacle(shapegroup)
        return tvco

    @classmethod
    def create_from_path(cls, path, t0, velocity, dt, offset=0):
        arclen_offset = offset
        arclen_current = arclen_offset
        segment_idx = 0
        arclen_idx = 0
        vertices = []
        velocity = velocity.copy()

        def interpolate(pt1, pt2, d):
            dist = np.linalg.norm(pt2 - pt1)
            return (1 - d / dist) * pt1 + d / dist * pt2

        # begin legacy code
        """
        def update_segment_idx():
            nonlocal arclen_current
            nonlocal segment_idx
            nonlocal arclen_idx
            nonlocal path
            while (segment_idx < len(path)-1 and
                   arclen_idx+np.linalg.norm(path[segment_idx+1]-path[segment_idx]) <= arclen_current):
                arclen_idx += (np.linalg.norm(path[segment_idx + 1] -
                               path[segment_idx]))
                segment_idx = segment_idx+1
                if segment_idx == len(path):
                    raise NameError(('Cannot create trajectory from path and '
                                     'velocity profile, end of path reached'))
        """

        # update_segment_idx()

        # end legacy code
        while ((segment_idx < len(path) - 1) and
                   (arclen_idx + np.linalg.norm(path[segment_idx + 1]
                                                - path[segment_idx])
                        <= arclen_current)):
            arclen_idx += np.linalg.norm(path[segment_idx + 1] -
                                         path[segment_idx])
            segment_idx += 1
            if segment_idx == len(path):
                raise NameError(('Cannot create trajectory from path and '
                                 'velocity profile, end of path reached'))

        vertices.append(interpolate(path[segment_idx],
                                         path[segment_idx + 1],
                                         arclen_current - arclen_idx))
        for v in velocity[:-1]:
            arclen_current = arclen_current + v * dt

            # update_segment_idx()

            # legacy
            while (segment_idx < len(path) - 1 and
                       (arclen_idx + np.linalg.norm(path[segment_idx + 1] -
                                                    path[segment_idx]) <= arclen_current)):
                arclen_idx += np.linalg.norm(path[segment_idx + 1] -
                                             path[segment_idx])
                segment_idx += segment_idx
                if segment_idx == len(path):
                    raise NameError(('Cannot create trajectory from path and '
                                     'velocity profile, end of path reached'))

            vertices.append(
                interpolate(path[segment_idx], path[segment_idx + 1],
                            arclen_current - arclen_idx))

        vertices = np.asarray(vertices)

        orientation = fvks.geometry.polyline.calculate_orientation_from_polyline(
            vertices)
        return cls._create_from_unpacked_states(vertices=vertices, t0=t0,
                                                orientation=orientation,
                                                velocity=velocity)
        #return cls(vertices=vertices, t0=t0, orientation=orientation,
        #           velocity=velocity)

    @classmethod
    def create_from_vertices(cls, vertices, t0, orientation=None,
                             velocity=None):
        vertices = vertices.copy()
        if velocity is not None:
            velocity = velocity.copy()
        if orientation is None:
            orientation = fvks.geometry.polyline.calculate_orientation_from_polyline(
                vertices)
        else:
            orientation = orientation.copy()
        return cls._create_from_unpacked_states(vertices=vertices, t0=t0,
                                                orientation=orientation,
                                                velocity=velocity)
#        return cls(vertices=vertices, t0=t0, orientation=orientation,
#                   velocity=velocity)
    def translate_rotate(self, translation, angle):
        for i in range(len(self.state_list)):
            self.state_list[i] = self._translate_rotate_state(translation, angle, self.state_list[i])

    def _translate_rotate_state(self, translation, angle, state):
        state = state._asdict()
        if StateTupleFactory.position in state.keys():
            if not type(state[StateTupleFactory.position]) == np.ndarray:
                raise Exception()
            else:
                t_m = fvks.geometry.transform.translation_rotation_matrix(
                    translation, angle)
                old_position = state[StateTupleFactory.position]
                new_position = t_m.dot(np.array(
                    [[old_position[0]], [old_position[1]], [1]]))
                state[StateTupleFactory.position] = np.array(
                    [new_position[0][0], new_position[1][0]])
        if StateTupleFactory.orientation in state.keys():
            if not is_exact(state[StateTupleFactory.orientation]):
                raise Exception
            else:
                state[StateTupleFactory.orientation] += angle
        return self.state_tuple(**state)


def collision_object_from_trajectory(trajectory, time_begin=-1, time_end=-1,
                                     CAR_WIDTH=2.0, CAR_LENGTH=4.0):
    position = list()
    orientation = list()
    for state in trajectory.state_list:
        if type(state.position) is not np.ndarray:
            raise Exception()
        if state.position.shape != (2,):
            raise Exception()
        position.append(state.position)
        if not is_exact(state.orientation):
            raise Exception()
        orientation.append(state.orientation)
    position = np.array(position)
    orientation = np.array(orientation)
    return pyfvks.collision.Trajectory(trajectory.t0, position, orientation,
                                       CAR_LENGTH, CAR_WIDTH)


def collision_object_from_trajectory_python(trajectory, time_begin=-1, time_end=-1,
                                     CAR_WIDTH=2.0, CAR_LENGTH=4.0):

    def interpolate_configurations(a_pos, a_orient, b_pos, b_orient):

        def reduce_angle(angle):
            angle = ((angle % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
            return angle if angle <= np.pi else angle - 2 * np.pi

        def angle_diff(a, b):
            """
            difference (-pi,pi) between two angles
            negative sign counterclockwise a->b
            positiv sign clockwise a->b
            """
            c = a - b
            if c > np.pi:
                c -= 2 * np.pi
            elif c < -np.pi:
                c += 2 * np.pi
            return c

        MAX_INTERP_DISTANCE = 1.0
        a_orient = reduce_angle(a_orient)
        b_orient = reduce_angle(b_orient)
        tangent = b_pos - a_pos
        distance = np.linalg.norm(tangent)
        orient_diff = angle_diff(a_orient, b_orient)
        interp_conf = [(a_pos, a_orient)]
        for i in range(1, int(distance / MAX_INTERP_DISTANCE) + 1):
            r = i * MAX_INTERP_DISTANCE / distance
            interp_conf.append((a_pos + r * tangent,
                                a_orient - r * orient_diff))
        interp_conf.append((b_pos, b_orient))
        return interp_conf


    tvo = pyfvks.collision.TimeVariantCollisionObject(trajectory.t0)
    for i in range(len(trajectory.state_list) - 1):
        state_c = trajectory.state_list[i]
        state_n = trajectory.state_list[i + 1]
        interp_conf = interpolate_configurations(state_c.position,
                                                 state_c.orientation,
                                                 state_n.position,
                                                 state_n.orientation)
        sg = pyfvks.collision.ShapeGroup()
        for i in interp_conf:
            sg.add_shape(pyfvks.collision.RectOBB(0.5*CAR_LENGTH,
                                                  0.5*CAR_WIDTH,
                                                  i[1], i[0][0], i[0][1]))
        tvo.append_obstacle(sg)

    """
    time_begin = time_begin if time_begin >= 0 else trajectory.t0
    time_end = time_end if time_end >= 0 else trajectory.t0 + len(trajectory.vertices) - 1
    tvo = pyfvks.collision_TimeVariantCollisionObject(time_begin)
    idx_traj = time_begin - trajectory.t0
    for t in range(time_begin, time_end):
        interp_conf = interpolate_configurations(
            trajectory.vertices[idx_traj],
            trajectory.orientation[idx_traj],
            trajectory.vertices[idx_traj + 1],
            trajectory.orientation[idx_traj + 1])
        sg = pyfvks.collision_ShapeGroup()
        for i in interp_conf:
            sg.addShape(pyfvks.collision_RectOBB(CAR_LENGTH_HALF,
                                                 CAR_WIDTH_HALF,
                                                 i[1], i[0][0], i[0][1]))
        tvo.appendObstacle(sg)
        idx_traj += 1
    """
    return tvo


