import numpy as np


def translation_rotation_matrix(translation, angle):
    """
    First translate homogeneous point, then rotate it around origin.
    """
    translation_matrix = np.array([[1, 0, translation[0]],
                                   [0, 1, translation[1]],
                                   [0, 0, 1]])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                [np.sin(angle), np.cos(angle), 0],
                                [0, 0, 1]])
    return rotation_matrix.dot(translation_matrix)


def to_homogeneous_coordinates(points):
    assert(len(points.shape) == 2)
    assert(points.shape[1] == 2)
    return np.hstack((points, np.ones((len(points), 1))))


def from_homogeneous_coordinates(points):
    assert(len(points.shape) == 2)
    assert(points.shape[1] == 3)
    return points[:, 0:2]
