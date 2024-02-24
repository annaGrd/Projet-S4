import tarfile
from time import time

from noeud import Noeud
from random import uniform
import numpy as np
from random_sampling import ellipse_sampling, uniform_sampling, line_sampling
from utils_grid import norme, mkeXsi
from constants import alpha, beta, kmax, rs, vFree, edge
from math import pi


def closest_node(xrand, cell):
    Xsi = mkeXsi(xrand, cell)

    if not Xsi : return None

    minimumArg = Xsi[0]
    minimumValue = norme(minimumArg, xrand)

    for xi in Xsi:
        value = norme(xi, xrand)
        if minimumValue > value:
            minimumArg = xi
            minimumValue = value

    return minimumArg


class Tree:
    def __init__(self, noeuds=None, xa=Noeud(), xgoal=Noeud()):
        if noeuds is None:
            noeuds = list()
        self.Et = list()
        self.Vt = noeuds
        self.Qs = list()
        self.Qr = list()
        self.traj = list()
        self.mem = list()
        self.xa = xa  # position du drone
        self.xgoal = xgoal  # faire choisir un point d'arrivée

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

    def remove_link(self, x0, x1):
        if (x0, x1) in self.Et:
            self.Et.remove((x0, x1))
        else:
            self.Et.remove((x1, x0))

        indexX1 = x0.voisins[0].index(x1)
        del x0.voisins[0][indexX1]
        del x0.voisins[1][indexX1]

        indexX0 = x1.voisins[0].index(x0)
        del x1.voisins[0][indexX0]
        del x1.voisins[1][indexX0]

    def add_link(self, x0, x1, c0=None, c1=None):
        if c0 is None:
            c0 = x0.ci
        if c1 is None:
            c1 = x1.ci

        self.Et.append((x0, x1))  # faire un dictionnaire
        x1.voisins[0].append(x0)
        x1.voisins[1].append(c0)
        x0.voisins[0].append(x1)
        x0.voisins[1].append(c1)

    def expansion_and_rewiring(self):

        xrand = self.rand_node()

        xclosest = closest_node(xrand)

        if line(xclosest, xrand):
            Xnear = self.find_nodes_near(xrand)

            if len(Xnear) < kmax or norme(xclosest, xrand) > rs:
                self.add_node(xrand, xclosest, Xnear)
                self.Qr.insert(xrand, 0)
                """# Qr étant une pile, 
                on pourrait plutôt utiliser la fin de la liste, jsp si ça sera mieux
                - T'es sûr que Qr est une pile ? Dans l'algo 2, on rajoute des éléments par le début 
                et dans l'algo 4, par la fin. A la limite, Qs est une pile, mais Qr, je ne crois pas ??"""
            else:
                self.Qr.insert(xclosest, 0)

            self.rewire_random_node()
        self.rewire_from_root()

    def add_node(self, xnew, xclosest, Xnear, cell):  # tu comptes mettre les algos 3,4 et 5 dans ces trois fonctions ?
        xmin = xclosest
        cmin = xclosest.cost(self.traj[0], self.xgoal) + norme(xclosest, xnew)

        for xnear in Xnear:

            cnew = xnear.cost(self.traj[0], self.xgoal) + norme(xnear, xnew)

            if cnew < cmin and line(xnear, xnew):
                cmin = cnew
                xmin = xnear

        self.Vt.append(xnew)
        self.add_link(xmin)

        # Ajout du nœud dans la case correspondante dans la grille
        qx = xnew.x // edge
        qy = xnew.y // edge
        qz = xnew.z // edge
        cell[qx][qy][qz].append(xnew)

    def rewire_random_node(self):
        t = time()
        while t - time() < 100 and len(self.Qr) > 0:  # Temps arbitraire

            xr = self.Qr.pop(0)
            Xnear = self.find_nodes_near(xr)

            for xnear in Xnear:
                cold = xnear.cost(self.traj[0], self.xgoal)
                cnew = xr.cost(self.traj[0], self.xgoal) + norme(xr, xnear)

                if cnew < cold and line(xr, xnear):
                    pa = xnear.parent(self.traj[0], self.xgoal)

                    self.add_link(xr, xnear, cnew)
                    self.remove_link(xnear, pa)

                    if xnear not in self.Qr:
                        self.Qr.append(xnear)

    def rewire_from_root(self):

        if not self.Qs:
            self.Qs.append(self.traj[0])
            self.mem = list()

        t = time()
        while t - time() < 100 and self.Qs:

            xs = self.Qs.pop(0)
            Xnear = self.find_nodes_near(xs)

            for xnear in Xnear:

                cold = xnear.cost(self.traj[0], self.xgoal)
                cnew = xnear.cost(self.traj[0], self.xgoal) + norme(xs, xnear)

                if cnew < cold and line(xs, xnear):
                    pa = xnear.parent(self.traj[0], self.xgoal)

                    self.add_link(xs, xnear, cnew)
                    self.remove_link(xnear, pa)

                if xnear not in self.mem:  # à ajuster quand on gérera les obstacles dynamiques
                    self.Qs.append(xnear)
                    self.mem.append(xnear)

    def find_nodes_near(self, x):
        Xnear = mkeXsi(x)
        epsilon = ((vFree * kmax) / pi * len(self.Vt)) ** .5

        if epsilon < rs: epsilon = rs

        for xnear in Xnear:
            if norme(x, xnear) < epsilon: Xnear.append(xnear)

        return Xnear
