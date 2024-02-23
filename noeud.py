import numpy as np
from math import inf

from utils_grid import norme


class Noeud:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.block = False  # être exploré ou non
        self.marqueur = False  # pour fc, savoir si bloqué pour fc
        self.ci = float("inf")
        self.voisins = [np.array(list()), np.array(list())]  # fusionner

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
    
        for voisin in voisins:
    
            if voisin[1] < cmin:
    
                cmin = voisin[1]
                pa = voisin[0]
    
        return pa

    def cost(self, xo, xgoal):

        """ Pas des plus optimisés pour le moment, recalcule le ci à chaque fois.
        En ayant connaissance des changements en amont, on n'aurait pas à tout recalculer.
        Utiliser blocked ?
        """

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
