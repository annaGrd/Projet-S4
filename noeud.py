import numpy as np
from math import inf

from constants import l_min
from utils_grid import norme, inGrid


class Noeud:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.block = False  # bloqué par obstacle dynamique
        self.already_seen = False  # vu ou non par l'algo 6 pour le xgoal actuel
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
        L'idée ici, c'est qu'à chaque fois qu'il y a une modification dans les liens entre les noeuds, on calcule
        récursivement les couts des noeuds qui descendent du nouveau nœud parent (par ex xs ou xr pour les algos 4 et 5)

        Pour les obstacles, il suffira de mettre le nœud bloqué avec un coup infini et d'exécuter cette fonction

        Dans le papier, ils recalculent le cout uniquement quand il y a besoin, c'est vrai qui c'est mieux mais, si cette
        technique suffit, ce sera plus simple
        """

        for x in self.voisins:
            if x.ci < self.ci: continue

            potentialNewCost = self.ci + norme(x, self)
            if potentialNewCost != x.ci:
                x.ci = potentialNewCost
                x.recalculate_child_costs()

    def fc(self, xgoal):

        if self.already_seen:
            return inf

        else:
            return self.ci + norme(self, xgoal)

    def bestChild(self, xo, xgoal):

        pa = self.parent(xo)
        v = [x for x in self.voisins if (pa is None or x != pa)]
        fcs = [x.fc(xgoal) for x in v]

        fc_min = fcs[0]
        best_child = v[0]

        for voisin_idx in range(len(v)):

            if fcs[voisin_idx] < fc_min:
                best_child = v[voisin_idx]
                fc_min = fcs[voisin_idx]

        return best_child

    def line(self, other):
        dist = norme(self, other)
        numb_try = int(dist)

        coord1 = np.array([self.x, self.y, self.z])
        coord2 = np.array([other.x, other.y, other.z])
        coord_unitaire = (coord2 - coord1) / numb_try

        for i in range(numb_try):
            coord_on_line = coord1 + i * coord_unitaire
            if not inGrid(Noeud(coord_on_line[0], coord_on_line[1], coord_on_line[2])):
                return False
        return True

    def unblock(self, xo):
        """
        Debloque le noeuds ainsi que ses ancetres (utilise quand on ajoute une node ou qu'on rewire)
        """
        self.already_seen = False
        pa = self.parent(xo)
        if pa is not None:
            pa.unblock(xo)