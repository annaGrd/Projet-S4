from noeud import Noeud
from constants import update_time, safety_radius, edge
from utils_grid import list_indices_at_range, norme

"""obstacles est une liste contenant les obstacles dynamiques de la forme [x, y, z, vitesse]"""


def update_block(T, obstacles):

    # on récupère tous les noeuds marqués
    blocked = list()
    for x in T.Vt:
        if x.block:
            blocked.append(x)

    # on récupère tous les noeuds dans la range rb de chaque obstacle
    for obs in obstacles:
        x_inrange = calcul_inrange(T, obs)

        # si le nœud est déjà marqué, il le restera, sinon, il le devient
        for x in x_inrange:
            if x.block: blocked.remove(x)
            x.block = True

    # il ne reste plus que les noeuds qui ne sont plus en range d'un obstacle
    for x in blocked:
        x.block = False


def calcul_inrange(T, obs):
    rb = update_time * obs[3] + safety_radius

    qx = int(obs[0] // edge)
    qy = int(obs[1] // edge)
    qz = int(obs[2] // edge)

    if qx == T.nbcellx: qx -= 1
    if qy == T.nbcelly: qy -= 1
    if qz == T.nbcellz: qz -= 1
    node_inrange = list(T.cell[qx, qy, qz])

    gap = list_indices_at_range(1)
    for abscissa, ordinate, altitude in gap:
        if not (0 <= qx + abscissa < T.nbcellx and 0 <= qy + ordinate < T.nbcelly and 0 <= qz + altitude < T.nbcellz): continue
        cell = T.cell[qx + abscissa][qy + ordinate][qz + altitude]
        node_inrange.extend(cell)

    for x in node_inrange:
        if norme(x, Noeud(obs[0], obs[1], obs[2])) > rb: node_inrange.remove(x)

    return node_inrange