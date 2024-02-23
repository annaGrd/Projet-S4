import numpy as np


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
