import tarfile
from time import time

from noeud import Noeud
from random import uniform
import numpy as np
from random_sampling import ellipse_sampling, uniform_sampling, line_sampling
from utils_grid import norme, cells, list_indices_at_range
from constants import alpha, beta, kmax, rs, vFree, edge, rg, k
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
        self.xa = xa  # position du drone
        self.root = xa  # racince de l'arbre
        self.xgoal = xgoal  # faire choisir un point d'arrivée
        self.cell = cells()
        self.nbcellx = len(self.cell)
        self.nbcelly = len(self.cell[0])
        self.nbcellz = len(self.cell[0][0])
        self.restart = False
        self.rewire_radius = .0

    def rand_node(self):
        xgoal = self.xgoal
        xo = self.root
        Pr = uniform(0, 1)
        IsGoalReached, xclose = self.goal_reached()

        if Pr > 1 - alpha and not IsGoalReached and xclose.line(xgoal):
            return line_sampling(xclose, xgoal)

        elif Pr <= (1 - alpha) / beta or not IsGoalReached:
            return uniform_sampling(self)

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
            else:
                self.Qr.insert(0, xclosest)

            self.rewire_random_node()
        self.rewire_from_root()

    def add_node(self, xnew, xclosest, Xnear):
        xmin = xclosest
        cmin = xclosest.ci + norme(xclosest, xnew)

        for xnear in Xnear:

            if norme(xnew, xnear) > rs:
                continue

            cnew = xnear.ci + norme(xnear, xnew)

            if cnew < cmin and xnear.line(xnew):
                cmin = cnew
                xmin = xnear

        self.Vt.append(xnew)
        self.add_link(xnew, xmin)
        self.add_node_to_cell(xnew)
        xnew.ci = xmin.ci + norme(xmin, xnew)
        xnew.unblock()

    def rewire_random_node(self):
        t = time()
        while t - time() < .01 and len(self.Qr) > 0:  # Temps arbitraire

            xr = self.Qr.pop(0)
            Xnear = self.find_nodes_near(xr)

            for xnear in Xnear:

                cold = xnear.ci
                cnew = xr.ci + norme(xr, xnear)

                if cnew < cold and xr.line(xnear):
                    pa = xnear.parent()

                    self.add_link(xr, xnear)
                    self.remove_link(xnear, pa)
                    xr.recalculate_child_costs()

                    xr.unblock()
                    xnear.unblock()

                    self.Qr.append(xnear)

    def rewire_from_root(self):

        if not self.Qs:
            self.Qs.append(self.root)

        t = time()
        while t - time() < .01 and self.Qs:
            xs = self.Qs.pop(0)
            Xnear = self.find_nodes_near(xs)

            for xnear in Xnear:

                cold = xnear.ci
                cnew = xnear.ci + norme(xs, xnear)

                if cnew < cold and xs.line(xnear):
                    pa = xnear.parent()

                    self.add_link(xs, xnear)
                    self.remove_link(xnear, pa)
                    xs.recalculate_child_costs()

                    xs.unblock()
                    xnear.unblock()

                if xnear not in self.Qs and self.restart:
                    self.Qs.append(xnear)
            self.rewire_radius = xs.ci

    def find_nodes_near(self, x):
        Xsi = self.mkeXsi(x)
        epsilon = vFree ** (1/3) * ((3*kmax)/(4*pi*len(self.Vt))) ** (1/3)  # Maintenant qu'on est en 3D, mu(B_epsilon) = 4/3*pi*epsilon**3
        # vFree est sorti de la racine principale pour éviter les overflow errors
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
        while extraRadius <= 2 and radius <= max(self.nbcellx, self.nbcelly, self.nbcellz): # Nombre arbitraire, designe le nombre de couches supplementaires a prendre en compte apres avoir trouve une node
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
        return norme(self.xgoal, xclose) < rg and xclose.line(self.xgoal), xclose

    def deadEnd(self, x):

        if len(x.voisins) < 2 and x != self.root:  # si n'a qu'un voisin (juste son parent)
            return True

        pa = x.parent()
        for v in x.voisins:
            if pa is not None and v == pa:
                pass
            elif not v.already_seen:  # si un enfant n'est pas déjà vu
                return False

        return True  # tous les enfants ont déjà été vus

    def plan(self):
        if self.goal_reached()[0]:
            xclosest = self.closest_node(self.xgoal)
            path = [xclosest]
            while xclosest is not None:
                xclosest = xclosest.parent()
                path.insert(0, xclosest)
            self.traj = path[1:]
            # chemin toujours accessible ? check les ci et compare les arêtes

        else:
            path = [self.root]
            while not self.deadEnd(path[-1]) and len(path) < k:
                path.append(path[-1].bestChild(self.xgoal))

            path[-1].already_seen = True
            if not self.path_exists(self.traj) or norme(self.traj[-1], self.xgoal) > norme(path[-1], self.xgoal):
                self.traj = path  # l.11
                return self, True
            return self, False

    def path_exists(self, path):
        """
        Verifie que le chemin existe toujours
        """

        for idx in range(len(path) - 1):
            if (path[idx], path[idx + 1]) not in self.Et and (path[idx + 1], path[idx]) not in self.Et:
                return False

        return True