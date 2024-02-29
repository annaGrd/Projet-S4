from dynamic import dynamic_obstacles
from ref_change import reference_change
from random import uniform
import numpy as np
from utils_grid import inGrid, norme
from noeud import Noeud
from constants import X, rs


def ellipsoid(x, y, z, a, b):
    return (x**2 + y**2) / b**2 + z**2 / a**2 <= 1


def reduce_distance_to_rs(x0, x1):
    """
    Retourne des coordonnées pour que le nœud soit au maximum à une distance rs du noeud le plus proche
    """
    coordinatesX1 = np.array([x1.x, x1.y, x1.z])
    coordinatesX0 = np.array([x0.x, x0.y, x0.z])

    coordinatesNewX = (coordinatesX1 - coordinatesX0) / norme(x1, x0) * rs + coordinatesX0
    newX = Noeud(coordinatesNewX[0], coordinatesNewX[1], coordinatesNewX[2])
    return newX


def ellipse_sampling(T, root, goal, a, b):
    """
    Renvoie des coordonnées aléatoires dans une ellipsoïde paramétrée autour de xo et xgoal
    """
    x = uniform(-b, b)
    y = uniform(-b, b)
    z = uniform(-a, a)
    while not ellipsoid(x, y, z, a, b):
        x = uniform(-b, b)
        y = uniform(-b, b)
        z = uniform(-a, a)

    new = np.array([x, y, z])

    new = reference_change(root, goal, new)
    newNode = Noeud(new[0], new[1], new[2])

    if not inGrid(newNode, dynamic_obstacles): return ellipse_sampling(T, root, goal, a, b)

    closestNode = T.closest_node(newNode)
    if norme(newNode, closestNode) > rs:
        newNode = reduce_distance_to_rs(closestNode, newNode)

    if not inGrid(newNode, dynamic_obstacles): return ellipse_sampling(T, root, goal, a, b)

    return newNode


def line_sampling(xclose, xgoal):
    """
    Retourne un point aléatoire sur le segment entre xclose et xgoal
    """
    x1 = np.array([xclose.x, xclose.y, xclose.z])
    goal = np.array([xgoal.x, xgoal.y, xgoal.z])

    r = uniform(0, 1)
    new = (1-r) * x1 + r * goal
    newNode = Noeud(new[0], new[1], new[2])

    if norme(newNode, xclose) > rs:
        newNode = reduce_distance_to_rs(xclose, newNode)

    if not inGrid(newNode, dynamic_obstacles): return line_sampling(xclose, xgoal)

    return newNode


def uniform_sampling(T):
    """
    Retourne des coordonnées aléatoires dans la grille
    """
    x = uniform(X[0][0], X[0][1])
    y = uniform(X[1][0], X[1][1])
    z = uniform(X[2][0], X[2][1])

    newNode = Noeud(x, y, z)
    closestNode = T.closest_node(newNode)
    if norme(newNode, closestNode) > rs:
        newNode = reduce_distance_to_rs(closestNode, newNode)

    if not inGrid(newNode, dynamic_obstacles): return uniform_sampling(T)

    return newNode
