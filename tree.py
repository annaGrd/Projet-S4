from noeud import Noeud
from random import uniform
import numpy as np
from random_sampling import ellipse_sampling, uniform_sampling, line_sampling
from utils_grid import norme
from constants import alpha, beta, kmax, rs, vFree
from math import pi


def closest_node(Xsi, xrand):
    minimumArg = Xsi[0]
    minimumValue = norme(minimumArg, xrand)

    for xi in Xsi:
        value = norme(xi, xrand)
        if minimumValue > value:
            minimumArg = xi
            minimumValue = value

    return minimumArg

class Tree:
    def __init__(self, noeuds=None, xa = Noeud(), xgoal = Noeud()):
        if noeuds is None:
            noeuds = list()
        self.Et = list()
        self.Vt = noeuds
        self.Qs = list()
        self.Qr = list()
        self.traj = list()
        self.mem = list()
        self.xa = xa # position du drone
        self.xgoal = xgoal # faire choisir un point d'arriver

    def rand_node(self):
        xgoal = self.xgoal
        xo = self.traj[0]
        Pr = uniform(0, 1)
        xclose = self.traj[-1]

        if Pr > 1 - alpha:

            root = np.array([xo.x, xo.y, xo.z])
            goal = np.array([xgoal.x, xgoal.y, xgoal.z])

            return line_sampling(root, goal)

        elif Pr <= (1 - alpha) / beta or xclose != xgoal:

            return uniform_sampling()

        else:
            cmin = norme(xo, xgoal)
            cbest = xgoal.ci  # est-ce bien la distance entre xo et xgoal, pas sur
            a = cbest / 2
            b = (cbest ** 2 + cmin ** 2) ** 0.5 / 2

            root = np.array([xo.x, xo.y, xo.z])
            goal = np.array([xgoal.x, xgoal.y, xgoal.z])

            return ellipse_sampling(root, goal, a, b)
        
    def expansion_and_rewriting(self):
        
        xrand = self.rand_node()

        Xsi = []
        xclosest = closest_node(Xsi, xrand)

        if line(xclosest, xrand):
            Xnear = self.find_nodes_near(xrand, Xsi)

            if len(Xnear) < kmax or norme(xclosest, xrand) > rs:
                self.add_node(xrand, xclosest, Xnear)
                self.Qr.insert(xrand,0)  # Qr étant une pile, on pourra plutot utiliser la fin de la liste, jsp si ça sera mieux
            else:
                self.Qr.insert(xclosest, 0)

            self.rewire_random_node()
        self.rewire_from_root()

    def add_node(self, xrand, xclosest, Xnear):
        pass

    def rewire_random_node(self):
        pass

    def rewire_from_root(self):
        pass

    def find_nodes_near(self, x, Xsi):
        Xnear = list()
        epsilon = ((vFree * kmax) / pi * len(self.Vt)) ** .5

        if epsilon < rs: epsilon = rs

        for xnear in Xsi:
            if norme(x, xnear) < epsilon: Xnear.append(xnear)