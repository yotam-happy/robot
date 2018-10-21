import numpy as np
import math


def get_translation_transform(translation):
    def _f(coords):
        coords[:, :] = coords + translation
    return _f


def get_scale_transform(scale):
    def _f(coords):
        coords[:, :] = coords * scale
    return _f


def get_rotation_transform(axis, angle, origin=np.zeros(3)):
    matrix = _rotation_matrix(axis, angle)
    def _f(coords):
        coords[:, :] = coords - origin
        coords[:, :] = np.dot(matrix, coords.T).T
        coords[:, :] = coords + origin
    return _f

def get_fake_perspective_transform(focal_distance):
    def _f(coords):
        coords[:, 0:2] = coords[:, 0:2] * focal_distance / coords[:, 2].reshape((coords.shape[0], 1))
    return _f


def _rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
