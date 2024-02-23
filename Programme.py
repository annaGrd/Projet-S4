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

# Algo 3


def addNode(T, xnew, xclosest, Xnear):

    xmin = xclosest
    cmin = cost(T, xclosest) + norme(xclosest, xnew)

    for xnear in Xnear:

        cnew = cost(T, xnear) + norme(xnear, xnew)

        if cnew < cmin and line(xnear, xnew):
            cmin = cnew
            xmin = xnear

    T.Vt.append(xnew)
    T.Et.append((xmin, xnew))  # faire un dictionnaire
    xnew.voisins[0].append(xmin)
    xnew.voisins[1].append(xmin.ci)
    xmin.voisins[0].append(xnew)
    xmin.voisins[1].append(xnew.ci)

    # Ajout du nœud dans la case correspondante dans la grille
    qx = xnew.x // edge
    qy = xnew.y // edge
    qz = xnew.z // edge
    cell[qx][qy][qz][3].append(xnew)

    return None


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

# Mettre en commun des bouts d'algo 4 et 5.


def rewireRandNode(T):

    t = time()
    while t - time() < 100 or len(T.Qr) > 0:  # Temps arbitraire

        xr = T.Qr.pop(0)
        Xnear = T.find_nodes_near(xr)

        for xnear in Xnear:
            cold = cost(T, xnear)
            cnew = cost(T, xr) + norme(xr, xnear)

            if cnew < cold and line(xr, xnear):
                T.Et.append((xr, xnear))
                pa = parent(T, xnear)

                if (xnear, pa) in T.Et:
                    T.Et.remove((xnear, pa))
                else:  # volontairement un else pour déclencher une erreur si cette arête n'existait pas
                    T.Et.remove((pa, xnear))

                if xnear not in T.Qr:  # cette condition est censée être toujours vraie
                    T.Qr.append(xnear)
                xr.voisins[0].append(xnear)
                xr.voisins[1].append(cnew)
    return T

# Algo 5


def rewireRoot(T):

    if len(T.Qs) == 0:
        T.Qs.append(T.traj[0])
        T.mem = list()

    t = time()
    while t - time() < 100 or len(T.Qs) > 0:

        xs = T.Qs.pop(0)
        Xnear = T.find_nodes_near(xs)

        for xnear in Xnear:

            cold = cost(T, xnear)
            cnew = cost(T, xs) + norme(xs, xnear)

            if cnew < cold and line(xs, xnear):

                T.Et.append((xs, xnear))
                pa = parent(T, xnear)

                if (xnear, pa) in T.Et:
                    T.Et.remove((xnear, pa))

                else:
                    T.Et.remove((pa, xnear))

                xs.voisins[0].append(xnear)
                xs.voisins[1].append(cnew)

            if xnear not in T.mem:  # à ajuster quand on gérera les obstacles dynamiques
                T.Qs.append(xnear)
                T.mem.append(xnear)

    return T

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
