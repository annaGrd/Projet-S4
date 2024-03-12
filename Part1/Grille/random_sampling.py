from ref_changes import reference_change
from random import uniform, randint
import numpy as np
from utils_grid import inGrid, norme
from Part1.MainAndClasses.noeud import Noeud
from Part1.MainAndClasses.constants import X, rs, edge


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

    if not inGrid(newNode): return ellipse_sampling(T, root, goal, a, b)

    closestNode = T.closest_node(newNode)
    if norme(newNode, closestNode) > rs:
        newNode = reduce_distance_to_rs(closestNode, newNode)

    if not inGrid(newNode): return ellipse_sampling(T, root, goal, a, b)

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

    if not inGrid(newNode): return line_sampling(xclose, xgoal)

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

    if not inGrid(newNode): return uniform_sampling(T)
    return newNode


def biased_sampling(T):
    """
    Retourne un noeud aléatoire dans une zone spécifique de l'espace.
    Choix de l'option 5 : On fait une pondération de toutes les cases dans cell en fonction du nombre de noeuds
    qu'elles contiennent. Moins elles contiennent des noeuds, plus elles ont de chances d'être choisies
    """
    if len(T.Vt) == 1:
        return uniform_sampling(T)
    lx = X[0][1]
    ly = X[1][1]
    lz = X[2][1]

    nbCellsx, rx = divmod(lx, edge)
    nbCellsy, ry = divmod(ly, edge)
    nbCellsz, rz = divmod(lz, edge)
    if rx: nbCellsx += 1
    if ry: nbCellsy += 1
    if rz: nbCellsz += 1

    cell = [[i, j, k] for i in range(int(nbCellsx)) for j in range(int(nbCellsy)) for k in range(int(nbCellsz))]
    weighting = list()
    nb_min = len(T.Vt)
    found_min = None
    for coord in cell:
        length_cell = len(T.cell[coord[0]][coord[1]][coord[2]])
        weighting.extend([coord for _ in range(len(T.Vt)-length_cell)])
        if nb_min >= length_cell and length_cell:
            found_min = coord
            nb_min = length_cell
    found_node = False
    while weighting:
        rg = randint(0, len(weighting)-1)
        chosen_cell = weighting[rg]
        new_node = try_in_cell(T, chosen_cell, 0)
        if new_node == Noeud():
            weighting.remove(chosen_cell)
        else:
            found_node = True
            break
    """
    Condition activée si on n'a vraiment pas chance, c'est-à-dire qu'on a tenté de mettre dix noeuds au pif
    dans toutes les cases et ils n'étaient pas dans la grille ou trop loin de l'arbre.
    """
    if not found_node:
        new_node = Noeud()
        while not new_node:
            new_node = try_in_cell(T, [found_min[0], found_min[1], found_min[2]], 0)


def try_in_cell(T, chosen_cell, attempt):
    """
    Tente de donner un noeud aléatoire dans chosen_cell
    """
    """
    Si la condition est vraie, c'est qu'on a galéré à trouver un noeud dans la case
    (peut-être qu'elle n'est pas accessible) donc on passe à la suivante.
    Cela peut être dû à une trop grande densité d'obstacles ou à une distance supérieure à rs par rapport à l'arbre.
    """
    if attempt > 9: return Noeud()  # combien de tentatives on veut

    x = uniform(chosen_cell[0], chosen_cell[0] + edge)
    y = uniform(chosen_cell[1], chosen_cell[1] + edge)
    z = uniform(chosen_cell[2], chosen_cell[2] + edge)
    newNode = Noeud(x, y, z)
    closestNode = T.closest_node(newNode)
    #if norme(newNode, closestNode) > rs:
        #newNode = reduce_distance_to_rs(closestNode, newNode)
    if not inGrid(newNode) : return try_in_cell(T, chosen_cell, attempt+1)
    return newNode