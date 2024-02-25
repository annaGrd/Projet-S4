from time import time
from tree import Tree
from noeud import Noeud
from constants import rprox, k
from utils_grid import norme


def main(xa, Xobs, xgoal):

    T = Tree([xa], Noeud(), Noeud())  # définir xa et xgoal
    t = time()

    while norme(T.xa, T.xgoal) > rprox:  # ? le drone proche de xgoal, à modifier en dynamique
        # On récupère le xa du drone et l'évolution de l'environnement
        if # xgoal has changed:
            for x in T.Vt:
                x.already_seen = False
        while time() - t < 100:  # Durée arbitraire
            T.expansion_and_rewiring()

        T, moving = plan(T)
        if norme(T.xa, T.traj[0]) <= rprox:  # Ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.traj[0].recalculate_child_costs()
            T.restart = True
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo si moving == True
    return  # ?

# Algo 6


def deadEnd(T, x):
    if len(x.voisins) < 2:  # si n'a qu'un voisin (juste son parent)
        return True

    else:
        pa = x.parent(T.traj[0])

        for v in x.voisins:
            if v == pa: pass
            elif not v.already_seen:  # si un enfant n'est pas déjà vu
                return False

        return True  # tous les enfants ont déjà été vus


def plan(T):

    if T.goal_reached():
        T.traj[-1] = T.xgoal
        # chemin toujours accessible ? check les ci et compare les arêtes

    else:
        path = [T.traj[0]]
        while not deadEnd(T, path[-1]) and len(path) < k:

            path.append(path[-1].bestChild(T.traj[0], T.xgoal))

        path[-1].already_seen = True
        T.traj = path  # l.11
        if norme(T.xa, T.xgoal) < norme(T.traj[-1], T.xgoal): return T, True
        return T, False