from utils_grid import norme
from constants import rs, vFree, kmax
from math import pi


def findNodesNear(T, x, Xsi):
    Xnear = list()
    epsilon = ((vFree * kmax)/pi*len(T.Vt))**.5

    if epsilon < rs: epsilon = rs

    for xnear in Xsi:
        if norme(x, xnear) < epsilon: Xnear.append(xnear)

    return Xnear

