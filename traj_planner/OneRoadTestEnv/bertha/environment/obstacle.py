"""
This module contains the generic class
TODO: make this class more generic and may be abstract
"""
import numpy as np
import matplotlib.pyplot as plt
import warnings
class Obstacle:
    def __init__(self, obstacle_points: np.ndarray, h: float=0.1, right: bool=False):
        self.points = np.copy(obstacle_points)
        self.h = h
        self.right = right


class StaticObstacle(Obstacle):

    def __init__(self, obstacle_points: np.ndarray, h: float=0.1, right: bool=False):
        super().__init__(obstacle_points, h, right)
        self.construct_polygon()

    def construct_affine_transformation_matrices(self):
        delta = self.points[2] - self.points[1]
        angle = np.arctan2(delta[1], delta[0])
        new_angle = (2.*np.pi - angle)
        self.rot_mat = np.array([
                                [np.cos(new_angle), -1.*np.sin(new_angle)],
                                [np.sin(new_angle), np.cos(new_angle)    ]
                                ])
        self.new_points = np.matmul(self.rot_mat,self.points.T).T # rotate points
        self.transl_mat = -1.*self.new_points[1] # translation
        self.new_points = self.new_points + self.transl_mat
        self.trans_mat = np.eye(3,dtype=float)
        self.trans_mat[:2,:2] = self.rot_mat
        self.trans_mat[2,:2] = self.transl_mat
        self.inv_rot_mat = self.rot_mat.T
        self.inv_trans_mat = np.eye(3,dtype=float)
        self.inv_trans_mat[:2,:2] = self.inv_rot_mat
        # https://stackoverflow.com/questions/2624422/efficient-4x4-matrix-inverse-affine-transform?noredirect=1&lq=1
        self.inv_trans_mat[2,:2] = -1.*np.matmul(self.inv_rot_mat,self.transl_mat.T).T
        return

    def construct_polygon(self):
        #print("original: ", t1, t2)
        self.construct_affine_transformation_matrices()
        self.t1 = (self.new_points[2] - self.new_points[0]) / (2*self.h)
        self.t2 = (self.new_points[3] - self.new_points[1]) / (2*self.h)
        self.t1 = self.t1 / np.linalg.norm(self.t1)
        self.t2 = self.t2 / np.linalg.norm(self.t2)
        self.p1p2_bar = self.new_points[2] - self.new_points[1]
        self.G = (self.p1p2_bar, self.t1, self.t2)
        return self.G

    def get_polygon(self):
        return self.G

    def get_transformation_matrix(self):
        return self.trans_mat

    def get_inv_tranfomation_matrix(self):
        return self.inv_trans_mat

    def compute_distance(self, trajectory: np.ndarray):
        new_traj = np.matmul(self.rot_mat, trajectory.T).T + self.transl_mat
        lambdas = (self.t1[1]*new_traj[:,1] - new_traj[:,0]) \
                    / ((self.t1[1] - self.t2[1]) * new_traj[:,1] + self.p1p2_bar[0])
        lambdas = (lambdas - lambdas.min(0)) / lambdas.ptp(0)
        plambdas = np.multiply(self.new_points[2], lambdas[:,np.newaxis])\
                   + np.multiply(self.new_points[1], (1.-lambdas)[:,np.newaxis])
        nlambdas = np.matmul(self.inv_rot_mat, (new_traj - plambdas).T).T + self.transl_mat
        dists = np.linalg.norm(nlambdas, axis=1)
        #plt.figure()
        #plt.plot(self.new_points[1, 0], self.new_points[1, 1], 'o', label='p1')
        #plt.plot(self.new_points[2, 0], self.new_points[2, 1], 'o', label='p2')
        #plt.plot(self.new_points[1:3, 0], self.new_points[1:3, 1], '--')
        #plt.quiver(self.new_points[1, 0], self.new_points[1, 1], self.t1[0], self.t1[1], label='t1', angles='xy')
        #plt.quiver(self.new_points[2, 0], self.new_points[2, 1], self.t2[0], self.t2[1], label='t2', angles='xy')
        #plt.plot(new_traj[0, 0], new_traj[0, 1], 'X', label='X')
        #plt.plot(new_traj[1, 0], new_traj[1, 1], 'X', label='X')
        #plt.plot(new_traj[0:2, 0], new_traj[0:2, 1], '--')
        #plt.quiver(plambdas[0,0], plambdas[0,1], nlambdas[0,0], nlambdas[0,1], label='nlambda', angles='xy')
        #plt.show()
        # with warnings.catch_warnings():
        #     warnings.filterwarnings('error')
        # try:
        #     if self.right:
        #         dists[new_traj[:, 1] < 0] = -1 * dists[new_traj[:, 1] < 0]
        #     else:
        #         dists[new_traj[:, 1] > 0] = -1 * dists[new_traj[:, 1] > 0]
        # except Warning:
        #     print('Raised!')

        return dists, nlambdas, plambdas

class DynamicObstacle(Obstacle):
    def construct_rings(self):
        pass
    def get_ring_centres(self):
        pass
class StaticObstacleHolder:
    def __init__(self, obstacle_points: np.ndarray, h: float=0.1, right: bool=False):
        self.obstacles = []
        padded_x = np.pad(obstacle_points[:,0], 1, mode='edge')
        padded_y = np.pad(obstacle_points[:,1], 1, mode='edge')
        padded = np.dstack((padded_x,padded_y))[0]
        self.obstacle_points = np.array([ padded[i:i+4,:] for i in range(len(padded)-3) ])
        for obs in self.obstacle_points:
            self.obstacles.append(StaticObstacle(obs, h, right))

    def get_obstacles(self):
        return self.obstacles
