import tarfile
from time import time

from noeud import Noeud
from random import uniform
import numpy as np
from random_sampling import ellipse_sampling, uniform_sampling, line_sampling
from utils_grid import norme, cells, list_indices_at_range
from constants import alpha, beta, kmax, rs, vFree, edge
from math import pi


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
        self.cell = cells()
        self.nbcellx = len(self.cell)
        self.nbcelly = len(self.cell[0])
        self.nbcellz = len(self.cell[0][0])

    def rand_node(self):
        xgoal = self.xgoal
        xo = self.traj[0]
        Pr = uniform(0, 1)
        IsGoalReached, xclose = self.goal_reached()

        if Pr > 1 - alpha:
            x1 = np.array([xclose.x, xclose.y, xclose.z])
            goal = np.array([xgoal.x, xgoal.y, xgoal.z])

            return line_sampling(x1, goal)

        elif Pr <= (1 - alpha) / beta or not IsGoalReached:
            return uniform_sampling()

        else:
            cmin = norme(xo, xgoal)
            cbest = xclose.ci  # est-ce bien la distance entre xo et xgoal, pas sur
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

        x0.voisins.remove(x1)
        x1.voisins.remove(x0)

    def add_link(self, x0, x1):

        self.Et.append((x0, x1))  # faire un dictionnaire
        x1.voisins.append(x0)
        x0.voisins.append(x1)

    def closest_node(self, xrand):
        Xsi = self.mkeXsi(xrand)

        if not Xsi: return None

        minimumArg = Xsi[0]
        minimumValue = norme(minimumArg, xrand)

        for xi in Xsi:
            value = norme(xi, xrand)
            if minimumValue > value:
                minimumArg = xi
                minimumValue = value

        return minimumArg

    def expansion_and_rewiring(self):

        xrand = self.rand_node()

        xclosest = self.closest_node(xrand)

        if xclosest.line(xrand):
            Xnear = self.find_nodes_near(xrand)

            if len(Xnear) < kmax or norme(xclosest, xrand) > rs:
                self.add_node(xrand, xclosest, Xnear)
                self.Qr.insert(0, xrand)
                """# Qr étant une pile, 
                on pourrait plutôt utiliser la fin de la liste, jsp si ça sera mieux
                - T'es sûr que Qr est une pile ? Dans l'algo 2, on rajoute des éléments par le début 
                et dans l'algo 4, par la fin. A la limite, Qs est une pile, mais Qr, je ne crois pas ??"""
            else:
                self.Qr.insert(0, xclosest)

            self.rewire_random_node()
        self.rewire_from_root()

    def add_node(self, xnew, xclosest, Xnear):
        xmin = xclosest
        cmin = xclosest.ci + norme(xclosest, xnew)

        for xnear in Xnear:

            cnew = xnear.ci + norme(xnear, xnew)

            if cnew < cmin and xnear.line(xnew):
                cmin = cnew
                xmin = xnear

        self.Vt.append(xnew)
        self.add_link(xnew, xmin)
        self.add_node_to_cell(xnew)
        xnew.ci = xmin.ci + norme(xmin, xnew)

    def rewire_random_node(self):
        t = time()
        while t - time() < .01 and len(self.Qr) > 0:  # Temps arbitraire

            xr = self.Qr.pop(0)
            Xnear = self.find_nodes_near(xr)

            for xnear in Xnear:
                cold = xnear.ci
                cnew = xr.ci + norme(xr, xnear)

                if cnew < cold and xr.line(xnear):
                    pa = xnear.parent(self.traj[0])

                    self.add_link(xr, xnear)
                    self.remove_link(xnear, pa)
                    xr.recalculate_child_costs()

                    self.Qr.append(xnear)

    def rewire_from_root(self):

        if not self.Qs:
            self.Qs.append(self.traj[0])
            self.mem = list()

        t = time()
        while t - time() < .01 and self.Qs:
            xs = self.Qs.pop(0)
            Xnear = self.find_nodes_near(xs)

            for xnear in Xnear:

                cold = xnear.ci
                cnew = xnear.ci + norme(xs, xnear)

                if cnew < cold and xs.line(xnear):
                    pa = xnear.parent(self.traj[0])

                    self.add_link(xs, xnear)
                    self.remove_link(xnear, pa)
                    xs.recalculate_child_costs()

                if xnear not in self.mem:  # à ajuster quand on gérera les obstacles dynamiques
                    self.Qs.append(xnear)
                    self.mem.append(xnear)

    def find_nodes_near(self, x):
        Xsi = self.mkeXsi(x)
        epsilon = vFree ** (1/3) * ((3*kmax)/(4*pi*len(self.Vt))) ** (1/3)  # Maintenant qu'on est en 3D, mu(B_epsilon) = 4/3*pi*epsilon**3
        # vFree est sorti de la racine principale pour eviter les overflow errors
        if epsilon < rs: epsilon = rs

        Xnear = []
        for xnear in Xsi:
            if norme(x, xnear) < epsilon: Xnear.append(xnear)

        return Xnear

    def mkeXsi(self, x):
        qx = int(x.x // edge)
        qy = int(x.y // edge)
        qz = int(x.z // edge)

        if qx == self.nbcellx: qx -= 1
        if qy == self.nbcelly: qy -= 1
        if qz == self.nbcellz: qz -= 1

        Xsi = list(self.cell[qx][qy][qz])
        if x in Xsi:
            Xsi.remove(x)

        radius = 0
        extraRadius = 0
        empty = True
        while extraRadius <= 2: # Nombre arbitraire, designe le nombre de couches supplementaires a prendre en compte apres avoir trouve une node
            radius += 1
            gap = list_indices_at_range(radius)
            for abscissa, ordinate, altitude in gap:
                """j'aimerais mettre une condition du type si l'indice n'est pas dans
                la liste, on ignore et on passe à la suite. N'étant point accoutumée 
                à l'usage des try and except, je t'invite à check si c'est correct"""

                if not (0 <= qx + abscissa < self.nbcellx and 0 <= qy + ordinate < self.nbcelly and 0 <= qz + altitude < self.nbcellz): continue

                cell = self.cell[qx + abscissa][qy + ordinate][qz + altitude]
                Xsi.extend(cell)
                if cell:
                    empty = False
            if not empty:
                extraRadius += 1
        return Xsi

    def add_node_to_cell(self, x):
        qx = int(x.x // edge)
        qy = int(x.y // edge)
        qz = int(x.z // edge)

        if qx == self.nbcellx: qx -= 1
        if qy == self.nbcelly: qy -= 1
        if qz == self.nbcellz: qz -= 1

        self.cell[qx][qy][qz].append(x)

    def goal_reached(self):
        xclose = self.closest_node(self.xgoal)
        return norme(self.xgoal, xclose) < .5 and xclose.line(self.xgoal), xclose
