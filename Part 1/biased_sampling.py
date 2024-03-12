from random import randint

from constants import edge, X
from noeud import Noeud
from random_sampling import try_in_cell
from utils_grid import list_indices_at_range


# Option 1 : On prend toutes les cases de la grille
def option1(T):
    lx = X[0][1]
    ly = X[1][1]
    lz = X[2][1]

    nbCellsx, rx = divmod(lx, edge)
    nbCellsy, ry = divmod(ly, edge)
    nbCellsz, rz = divmod(lz, edge)
    if rx: nbCellsx += 1
    if ry: nbCellsy += 1
    if rz: nbCellsz += 1

    return [[i, j, k] for i in range(int(nbCellsz)) for j in range(int(nbCellsy)) for k in range(int(nbCellsx))]


# Option 2 : On prend les cases autour de xgoal
def option2(T):
    cell = list()
    qx = int(T.xgoal.x // edge)
    qy = int(T.xgoal.y // edge)
    qz = int(T.xgoal.z // edge)

    if qx == T.nbcellx: qx -= 1
    if qy == T.nbcelly: qy -= 1
    if qz == T.nbcellz: qz -= 1
    gap = list_indices_at_range(3)  # à faire varier, arbitraire
    for abscissa, ordinate, altitude in gap:
        if not (
                0 <= qx + abscissa < T.nbcellx and 0 <= qy + ordinate < T.nbcelly and 0 <= qz + altitude < T.nbcellz): continue
        cell.append([qx + abscissa, qy + ordinate, qz + altitude])
    return cell


# Option 3 : On prend les cases autour de root
def option3(T):
    cell = list()
    qx = int(T.root.x // edge)
    qy = int(T.root.y // edge)
    qz = int(T.root.z // edge)

    if qx == T.nbcellx: qx -= 1
    if qy == T.nbcelly: qy -= 1
    if qz == T.nbcellz: qz -= 1
    gap = list_indices_at_range(3)  # à faire varier
    for abscissa, ordinate, altitude in gap:
        if not (
                0 <= qx + abscissa < T.nbcellx and 0 <= qy + ordinate < T.nbcelly and 0 <= qz + altitude < T.nbcellz): continue
        cell.append([qx + abscissa, qy + ordinate, qz + altitude])
    return cell


# Option 4 : On remplit en priorité les cases vides, puis celles avec 1 noeud, ...
def option4(T, cell):
    found_node = False
    nb_nodes = 0
    max_node = max([len(T.cell[coord[0][coord[1]][coord[2]]]) for coord in cell])  # nombre de noeuds maximal contenu dans une case de l'échantillon
    found_min = None
    while not found_node or nb_nodes <= max_node:
        potential_cells = list()  # contient la liste des cases qui ont nb_nodes noeuds dedans
        for coord in cell:
            if len(T.cell[coord[0][coord[1]][coord[2]]]) == nb_nodes:
                potential_cells.append(coord)
            if not found_min and nb_nodes and potential_cells:  # permet de choisir la case dans laquelle on bourrine au cas où
                found_min = coord
        if potential_cells:
            for chosen_cell in potential_cells:
                new_node = try_in_cell(T, chosen_cell, 0)
                if new_node != Noeud():
                    found_node = True
                    break
        nb_nodes += 1
    """
    Condition activée si on n'a vraiment pas chance, c'est-à-dire qu'on a tenté de mettre dix noeuds au pif
    dans toutes les cases et ils n'étaient pas dans la grille ou trop loin de l'arbre.
    On a donc pris une case qui est tout de même, peu remplie, mais comme elle contient au moins 1 noeud, c'est
    qu'on peut forcer jusqu'à en trouver un autre dedans. Un peu bourrin mais je ne vois pas d'autre solution :)
    """
    if not found_node:
        new_node = Noeud()
        while not new_node:
            new_node = try_in_cell(T, T.cell[found_min[0]][found_min[1]][found_min[2]], 0)
    return new_node


"""
Option 5 : On fait une pondération de toutes les cases dans cell en fonction du nombre de noeuds 
qu'elles contiennent. Moins elles contiennent des noeuds, plus elles ont de chances d'être choisies
"""
def option5(T, cell):
    weighting = list()
    nb_min = len(T.Vt)
    found_min = None
    for coord in cell:
        weighting.extend([coord for _ in range(len(T.Vt)-len(T.cell[coord[0]][coord[1]][coord[2]]))])
        if nb_min > len(T.cell[coord[0]][coord[1]][coord[2]]) and len(T.cell[coord[0]][coord[1]][coord[2]]):
            found_min = coord
    found_node = False
    while not found_node or weighting:
        rg = randint(0, len(weighting)-1)
        chosen_cell = weighting[rg]
        new_node = try_in_cell(T, chosen_cell, 0)
        if new_node == Noeud():
            weighting.remove(chosen_cell)
        else:
            found_node = True
    """
    Condition activée si on n'a vraiment pas chance, c'est-à-dire qu'on a tenté de mettre dix noeuds au pif
    dans toutes les cases et ils n'étaient pas dans la grille ou trop loin de l'arbre.
    """
    if not found_node:
        new_node = Noeud()
        while not new_node:
            new_node = try_in_cell(T, T.cell[found_min[0]][found_min[1]][found_min[2]], 0)
    return new_node