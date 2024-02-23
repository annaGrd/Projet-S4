from ref_change import reference_change
from random import uniform
import numpy as np
from utils_grid import inGrid
from noeud import Noeud
from constants import X


def ellipsoid(x, y, z, a, b):

    return (x**2 + y**2) / b**2 + z**2 / a**2 <= 1


def ellipse_sampling(root, goal, a, b):
    # recherche coordonnées, on peut le faire dès le début non ?
    x = uniform(-b, b)
    y = uniform(-b, b)
    z = uniform(-a, a)
    """c'est la merde, comment je teste que c'est dans la grille ?.
    Pour le moment, je le fais plus tard mais un peu contre-productif."""
    while not ellipsoid(x, y, z, a, b):
        x = uniform(-b, b)
        y = uniform(-b, b)
        z = uniform(-a, a)

    new = np.array([x, y, z])

    new = reference_change(root, goal, new)
    newNode = Noeud(new[0], new[1], new[2])
    if not inGrid(newNode): return ellipse_sampling(root, goal, a, b)

    return newNode


def line_sampling(root, goal):
    r = uniform(0, 1)
    new = (1-r)*root + r*goal
    newNode = Noeud(new[0], new[1], new[2])

    if not inGrid(newNode): return line_sampling(root, goal)

    return newNode


def uniform_sampling():
    x = uniform(X[0][0], X[0][1])
    y = uniform(X[1][0], X[1][1])
    z = uniform(X[2][0], X[2][1])

    newNode = Noeud(x, y, z)

    if not inGrid(newNode): return uniform_sampling()

    return newNode
