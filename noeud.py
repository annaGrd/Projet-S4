import numpy as np
from math import inf

from constants import l_min
from utils_grid import norme, inGrid


class Noeud:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.block = False  # être exploré ou non
        self.marqueur = False  # pour fc, savoir si bloqué pour fc
        self.ci = float("inf")
        self.voisins = [list(), list()]  # fusionner

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        else:
            return False

    def parent(self, xo, xgoal):

        # si on demande le parent de la racine
        if self == xo:
            return None

        voisins = self.voisins[0]
        ci = self.voisins[1]
        # prendre celui avec le ci plus petit que self

        for i in range(len(ci)):
            ci[i] = voisins[i].fc(xo, xgoal)

        cmin = ci[0]
        pa = voisins[0]

        for voisin_idx in range(len(voisins)):

            if ci[voisin_idx] < cmin:
                cmin = voisins[voisin_idx]
                pa = ci[voisin_idx]

        return pa

    def cost(self, xo, xgoal):

        """ Pas des plus optimisés pour le moment, recalcule le ci à chaque fois.
        En ayant connaissance des changements en amont, on n'aurait pas à tout recalculer.
        Utiliser blocked ?
        """

        if self == xo:
            self.ci = 0
            return 0

        pa = self.parent(xo, xgoal)
        if pa == xo:
            ci = norme(xo, self)
            self.ci = ci
            return ci

        elif pa.ci == inf or pa.marqueur:  # pas sûre d'avoir besoin des deux conditions
            self.ci = inf
            return inf

        else:
            ci = pa.cost() + norme(self, pa)
            self.ci = ci
            return ci

    def fc(self, xo, xgoal):

        # à modifier si on passe à un xgoal dynamique
        if self.marqueur:  # pas sûre que cette condition soit utile avec l'existence de cost()
            return inf

        else:
            return self.cost(xo, xgoal) + norme(self, xgoal)

    def bestChild(self, xo, xgoal):

        v = self.voisins[0]
        c = self.voisins[1]

        rg = v.index(self.parent(xo, xgoal))
        v.pop(rg)  # retire le parent
        c.pop(rg)  # et son coût
        xTempo = Noeud()
        xTempo.voisins[0] = v
        xTempo.voisins[1] = c

        return xTempo.parent(xo, xgoal)  # on récupère donc le deuxième meilleur voisin

    def line(self, other):
        dist = norme(self, other)
        numb_try = int(dist//l_min)

        coord1 = np.array([self.x, self.y, self.z])
        coord2 = np.array([other.x, other.y, other.z])
        coord_unitaire = (coord2-coord1)/numb_try

        for i in range(numb_try):
            coord_on_line = coord1 + i*coord_unitaire
            if not inGrid(Noeud(coord_on_line[0], coord_on_line[1], coord_on_line[2])):
                return False
            else:
                pass
        return True
