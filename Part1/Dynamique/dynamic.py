from random import randint
from math import inf

from Part1.MainAndClasses.noeud import Noeud, recalculate_costs
from Part1.MainAndClasses.constants import update_time, safety_radius, edge, ro
from Part1.Grille.utils_grid import list_indices_at_range, norme, inGrid
from Part1.Dynamique.dynamic_obstacles import get_dynamic_obstacles

"""
dynamic_obstacles est une liste contenant les obstacles dynamiques de la forme [x, y, z, vitesse]
A-t-on besoin de propager le x.block = True aux enfants d'un noeud dans la range rb ?
En soi, le ci des enfants qui passe à l'infini est suffisant, non ?
J'ai l'impression que c'est ce qu'ils expliquent dans la section Blocking Nodes by Dynamic Obstacles, p.5
C'est pas pour tout de suite mais, dans l'optique d'utiliser des obstacles dynamiques,
est-ce qu'on ne ferait pas prédire une trajectoire à notre programme avec un xgoal statique
et notre obstacle dynamique en temps que drone. Ainsi, on récupère la trajectoire et
paf, on a une trajectoire établie et qui marche, pour notre obstacle. Si l'obstacle dynamique
atteint l'extrémité de sa trajectoire, on peut le faire aller dans l'autre sens ou calculer
une nouvelle trajectoire avec un xgoal aléatoire.
"""


def update_block(T, dynamic_obstacles):
    # on récupère tous les noeuds marqués
    blocked = list()
    for x in T.Vt:
        if x.block:
            blocked.append(x)

    node_to_recalculate = list()
    # on récupère tous les noeuds dans la range rb de chaque obstacle
    for obs in dynamic_obstacles:
        if norme(T.xa, Noeud(obs[0], obs[1], obs[2])) < ro:  # l'obstacle doit être dans la range de considération du drone
            x_inrange = calcul_inrange(T, obs)

            # si le nœud est déjà marqué, il le restera, sinon, il le devient
            for x in x_inrange:

                if x == T.root:
                    continue

                if x.ci < T.rewire_radius:
                    T.Qs.clear()
                if x in blocked:
                    blocked.remove(x)
                x.block = True
                x.ci = float("inf")
                node_to_recalculate.append(x)

    recalculate_costs(node_to_recalculate)

    node_to_recalculate = list()  # A-t-on besoin de reset à ce moment-là où on peut juste append à l'ancienne liste
    # il ne reste plus que les noeuds qui ne sont plus en range d'un obstacle
    for x in blocked:
        x.block = False
        pa = x.parent
        if pa is not None and pa.ci < inf:  # si le parent n'était pas bloqué, on calcule le ci de notre nœud débloqué et de ses enfants
            x.ci = pa.ci + norme(x, pa)
            node_to_recalculate.append(x)
    recalculate_costs(node_to_recalculate)

    return None


def calcul_inrange(T, obs):
    """Choix d'un rb adapté à la vitesse.
    Dans le papier, j'imagine que les obstacles dynamiques se déplacent à la même vitesse,
    donc pas besoin de faire un rb adapté à chaque obstacle dynamique, mais j'imagine que
    qui peut le plus, peut le moins."""
    rb = update_time * obs[3] + safety_radius

    qx = int(obs[0] // edge)
    qy = int(obs[1] // edge)
    qz = int(obs[2] // edge)

    if qx == T.nbcellx: qx -= 1
    if qy == T.nbcelly: qy -= 1
    if qz == T.nbcellz: qz -= 1
    node_inrange = list(T.cell[qx][qy][qz])

    radius = 1
    node_inrange_in_radius = []
    while radius == 1 or node_inrange_in_radius:
        node_inrange_in_radius = []
        gap = list_indices_at_range(radius)
        for abscissa, ordinate, altitude in gap:
            if not (0 <= qx + abscissa < T.nbcellx and 0 <= qy + ordinate < T.nbcelly and 0 <= qz + altitude < T.nbcellz):
                continue

            node_inrange_in_radius.extend([x for x in T.cell[qx + abscissa][qy + ordinate][qz + altitude] if norme(x, Noeud(obs[0], obs[1], obs[2])) < rb])
        node_inrange.extend(node_inrange_in_radius)
        radius += 1

    return node_inrange

global timeBeforeChange
timeBeforeChange = 0

global xgoal
xgoal = Noeud(randint(0, 30), randint(0, 30), randint(0, 30))
while not inGrid(xgoal):
    xgoal = Noeud(randint(0, 30), randint(0, 30), randint(0, 30))


def update_goal_and_obstacles(T, t):
    """
    Récupère xgoal, les coordonnées des obstacles dynamiques,
    met à jour l'arbre et les marqueurs block.
    Retourne les changements
    """

    global timeBeforeChange
    global xgoal

    if t - timeBeforeChange > 10:
        xgoal = Noeud(randint(0, 30), randint(0, 30), randint(0, 30))
        while not inGrid(xgoal):
            xgoal = Noeud(randint(0, 30), randint(0, 30), randint(0, 30))
        timeBeforeChange = t

    change_xgoal = (xgoal != T.xgoal)
    T.xgoal = xgoal

    dynamicObstacles = get_dynamic_obstacles()
    update_block(T, dynamicObstacles)

    return change_xgoal, dynamicObstacles