import numpy as np
from math import inf

from utils_grid import norme, inGrid


class Noeud:
    def __init__(self, x=0.0, y=0.0, z=0.0, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.block = False  # bloqué par obstacle dynamique
        self.already_seen = False  # vu ou non par l'algo 6 pour le xgoal actuel
        self.ci = float("inf")
        self.childs = []
        self.parent = parent

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        else:
            return False

    def __bool__(self):
        return self != Noeud()

    def __repr__(self):
        return f"[{round(self.x,2)}, {round(self.y,2)}, {round(self.z,2)}]"

    def recalculate_child_costs(self, change_of_root=False):

        """
        L'idée ici, c'est qu'à chaque fois qu'il y a une modification dans les liens entre les noeuds, on calcule
        récursivement les couts des noeuds qui descendent du nouveau nœud parent (par ex xs ou xr pour les algos 4 et 5)

        Pour les obstacles, il suffira de mettre le nœud bloqué avec un coup infini et d'exécuter cette fonction

        Dans le papier, ils recalculent le cout uniquement quand il y a besoin, c'est vrai qui c'est mieux mais, si cette
        technique suffit, ce sera plus simple

        Si jamais on recalcule après un changement de racine, il ne faut plus prendre en compte l'ordre déjà établi par
        les ci, mais bien le nouvel ordre que l'on veut, d'où la modif de cet algo
        """

        for x in self.childs:
            if change_of_root:
                pa = x.parent
                if pa is not None:
                    x.childs.append(x.parent)
                x.parent = self
                x.childs.remove(self)

            potentialNewCost = self.ci + norme(x, self)
            if potentialNewCost != x.ci:
                x.ci = potentialNewCost
                x.recalculate_child_costs(change_of_root=change_of_root)

    def fc(self, xgoal):
        """
        Fonction de pondération pour déterminer le meilleur enfant en contournant les blocages
        dans des minimas locaux.
        """
        if self.already_seen:
            return inf

        else:
            return self.ci + norme(self, xgoal)

    def bestChild(self, xgoal):
        """
        Détermine l'enfant le plus pertinent pour notre prévision de trajectoire
        """
        v = [x for x in self.childs]
        fcs = [x.fc(xgoal) for x in v]

        fc_min = fcs[0]
        best_child = v[0]

        for voisin_idx in range(len(v)):

            if fcs[voisin_idx] < fc_min:
                best_child = v[voisin_idx]
                fc_min = fcs[voisin_idx]

        return best_child

    def line(self, other):
        """
        Indique si l'on peut tracer une arête entre deux noeuds
        """
        dist = norme(self, other)
        numb_try = int(dist)*100

        coord1 = np.array([self.x, self.y, self.z])
        coord2 = np.array([other.x, other.y, other.z])
        if numb_try:
            coord_unitaire = (coord2 - coord1) / numb_try

        for i in range(numb_try):
            coord_on_line = coord1 + i * coord_unitaire
            if not inGrid(Noeud(coord_on_line[0], coord_on_line[1], coord_on_line[2])):
                return False
        return True

    def not_seen(self):
        """
        Débloque le noeuds ainsi que ses ancêtres (utilise quand on ajoute une node ou qu'on rewire)
        """
        self.already_seen = False
        pa = self.parent
        if pa is not None:
            pa.not_seen()