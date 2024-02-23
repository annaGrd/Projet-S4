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


def cost(T, x):

    """ Pas des plus optimisés pour le moment, recalcule le ci à chaque fois.
    En ayant connaissance des changements en amont, on n'aurait pas à tout recalculer.
    Utiliser blocked ?

    Pas certaine que ma structure récursive fonctionne (retours appréciés)
    """

    pa = parent(T, x)
    if pa == T.traj[0]:
        ci = norme(T.traj[0], x)
        x.ci = ci
        return ci

    elif pa.ci == inf or pa.marqueur:  # pas sûre d'avoir besoin des deux conditions
        x.ci = inf
        return inf

    else:
        ci = cost(T, pa) + norme(x, pa)
        x.ci = ci
        return ci

# Algo 4

def fc(T, xc):

    # à modifier si on passe à un xgoal dynamique
    if xc.marqueur:  # pas sûre que cette condition soit utile avec l'existence de cost()
        return inf

    else:
        return cost(T, xc) + norme(xc, T.xgoal)


def parent(T, x):

    # si on demande le parent de la racine
    if x == T.traj[0]:
        return None

    voisins = x.voisins[0]
    ci = x.voisins[1]
    # prendre celui avec le ci plus petit que x

    for i in range(len(ci)):
        ci[i] = fc(T, voisins[i])

    cmin = ci[0]
    pa = voisins[0]

    for voisin in voisins:

        if voisin[1] < cmin:

            cmin = voisin[1]
            pa = voisin[0]

    return pa

# Algo 6


def bestChild(T, x):

    v = x.voisins[0]
    c = x.voisins[1]

    rg = v.index(parent(T, x))
    v.pop(rg)  # retire le parent
    c.pop(rg)  # et son coût
    xTempo = Noeud()
    xTempo.voisins.append(v)
    xTempo.ci.append(c)

    return parent(T, xTempo)  # on récupère donc le deuxième meilleur voisin


def deadEnd(T, x):
    if len(x.voisins[0]) < 2:  # si n'a qu'un voisin (juste son parent)
        return True

    else:
        pa = parent(T, x)
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

            path.append(bestChild(T, path[-1]))

        path[-1].block = True
        if fc(T, path[-1]) == fc(T, T.traj[-1]):

            # Comparer les arêtes, voir si elles ont changé. Regarder les ci, voir si l'un est infini
            """faire une diff de norme entre xa,xgoal et xk,goal"""

    return T
