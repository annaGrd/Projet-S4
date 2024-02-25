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
        self.voisins = []

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        else:
            return False

    def parent(self, xo):

        # si on demande le parent de la racine
        if self == xo:
            return None

        # prendre celui avec le ci plus petit que self

        pa = self.voisins[0]
        cmin = pa.ci

        for voisin in self.voisins:

            if voisin.ci < cmin:
                pa = voisin
                cmin = pa.ci

        return pa

    def recalculate_child_costs(self):

        """
        L'idée ici c'est qu'à chaque fois qu'il y a une modification dans les liens entre les noeuds, on calcule
        récursivement les couts des noeuds qui descendent du nouveau noeud parent (par ex xs ou xr pour les algo 4 et 5)

        Pour les obstacles il suffira de mettre le noeud bloqué avec un coup infini et d'exécuter cette fonction

        Dans le papier ils recalculent le cout uniquement quand il y a besoin, c'est vrai qui c'est mieux mais si cette
        technique suffit ce sera plus simple
        """

        for x in self.voisins:
            if x.ci < self.ci: continue

            potentialNewCost = self.ci + norme(x, self)
            if potentialNewCost != x.ci:
                x.ci = potentialNewCost
                x.recalculate_child_costs()

    def fc(self, xgoal):

        # à modifier si on passe à un xgoal dynamique
        if self.marqueur:  # pas sûre que cette condition soit utile avec l'existence de cost()
            return inf

        else:
            return self.ci + norme(self, xgoal)

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
        numb_try = int(dist // l_min)

        coord1 = np.array([self.x, self.y, self.z])
        coord2 = np.array([other.x, other.y, other.z])
        coord_unitaire = (coord2 - coord1) / numb_try

        for i in range(numb_try):
            coord_on_line = coord1 + i * coord_unitaire
            if not inGrid(Noeud(coord_on_line[0], coord_on_line[1], coord_on_line[2])):
                return False
            else:
                pass
        return True
