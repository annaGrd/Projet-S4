from math import inf
from time import time
from tree import Tree
from noeud import Noeud
from constants import edge, rprox
from utils_grid import norme, cells

cell = cells()


def mkeXsi(x):

    Xsi = list()
    qx = x.x // edge
    qy = x.y // edge
    qz = x.z // edge
    nodes = cell[qx][qy][qz][3]

    for e in nodes:
        if e != x:
            Xsi.append(e)


def main(xa, Xobs, xgoal):

    T = Tree([xa], Noeud(), Noeud())  # définir xa et xgoal
    t = time()

    while norme(T.xa, T.xgoal) > rprox:  # ? le drone proche de xgoal, à modifier en dynamique
        # On récupère le xa du drone et l'évolution de l'environnement
        while time() - t < 100:  # Durée arbitraire
            T.expansion_and_rewiring()

        T = plan(T)
        if norme(T.xa, T.traj[0]) <= rprox:  # Ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo
    return  # ?

# Algo 6

def deadEnd(T, x):
    if len(x.voisins[0]) < 2:  # si n'a qu'un voisin (juste son parent)
        return True

    else:
        pa = x.parent(T.traj[0], T.xgoal)
        enfant = x.voisins[0]  # référence partagée ?
        enfant.remove(pa)  # obtient liste des nœuds enfants

        for v in enfant:
            if not v.ci == inf:  # si un enfant n'est pas marqué
                return False

        return True  # tous les enfants sont marqués


def plan(T):

    if norme(T.traj[-1], T.xgoal) < rprox:
        T.traj[-1] = T.xgoal
        # chemin toujours accessible ? check les ci et compare les arêtes

    else:
        path = [T.traj[0]]
        while deadEnd(T, path[-1]):

            path.append(path[-1].bestChild(T.traj[0],T.xgoal))

        path[-1].block = True
        if path[-1].fc(T.traj[0], T.xgoal) == T.traj[-1].fc(T.traj[0], T.xgoal):

            # Comparer les arêtes, voir si elles ont changé. Regarder les ci, voir si l'un est infini
            """faire une diff de norme entre xa,xgoal et xk,goal"""

    return T
