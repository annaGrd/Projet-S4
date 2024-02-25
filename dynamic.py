from noeud import Noeud
from constants import update_time, safety_radius, edge, ro
from utils_grid import list_indices_at_range, norme
from math import inf

"""
obstacles est une liste contenant les obstacles dynamiques de la forme [x, y, z, vitesse]
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


def update_block(T, obstacles):
    # on récupère tous les noeuds marqués
    blocked = list()
    for x in T.Vt:
        if x.block:
            blocked.append(x)

    # on récupère tous les noeuds dans la range rb de chaque obstacle
    for obs in obstacles:
        if norme(T.xa, Noeud(obs[0], obs[1], obs[2])) < ro:  # l'obstacle doit être dans la range de considération du drone
            x_inrange = calcul_inrange(T, obs)

            # si le nœud est déjà marqué, il le restera, sinon, il le devient
            for x in x_inrange:
                if x.ci < T.rewire_radius:
                    T.Qs.clear()
                    T.restart = True
                if x.block: blocked.remove(x)
                x.block = True

    # il ne reste plus que les noeuds qui ne sont plus en range d'un obstacle
    for x in blocked:
        pa = x.parent(T.traj[0])
        if pa.ci < inf:  # si le parent n'était pas bloqué, on calcule le ci de notre nœud débloqué et de ses enfants
            x.ci = pa.ci + norme(x, pa)
            x.recalculate_child_costs()

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
        if not (
                0 <= qx + abscissa < T.nbcellx and 0 <= qy + ordinate < T.nbcelly and 0 <= qz + altitude < T.nbcellz): continue
        cell = T.cell[qx + abscissa][qy + ordinate][qz + altitude]
        node_inrange.extend(cell)

    for x in node_inrange:
        if norme(x, Noeud(obs[0], obs[1], obs[2])) > rb: node_inrange.remove(x)

    return node_inrange