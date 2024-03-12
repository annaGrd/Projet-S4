import numpy as np
from math import sin, cos, atan


def anglePhi(vect):
    if vect[0] != 0 and vect[1] != 0:
        return atan(vect[2] / ((vect[0]**2 + vect[1]**2)**0.5))
    else:
        return 0


def angleTheta(vect):
    if vect[0] != 0 and vect[1] != 0:
        return 2*atan(vect[1] / (vect[0] + (vect[0]**2 + vect[1]**2)**0.5))
    else:
        return 0


def rotaPhi(phi, vect):  # phi angle de latitude
    mat = np.array([[1, 0, 0], [0, sin(phi), -cos(phi)], [0, cos(phi), sin(phi)]])
    return np.matmul(vect, mat)


def rotaTheta(theta, vect):
    mat = np.array([[sin(theta), -cos(theta), 0], [cos(theta), sin(theta), 0], [0, 0, 1]])
    return np.matmul(vect, mat)


def reference_change(root, goal, new):
    """
    Translation et rotations pour se placer dans un repère centré sur le milieu
    du segment [root, goal], ces derniers étant sur l'axe Oz
    """
    mid = (root + goal)/2
    relative_root = root - mid

    theta = angleTheta(relative_root)
    phi = anglePhi(relative_root)

    new = rotaTheta(theta, new)
    new = rotaPhi(phi, new)
    new += mid

    return new
