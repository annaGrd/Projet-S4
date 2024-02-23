from noeud import Noeud
from random import uniform
import numpy as np
from random_sampling import ellipse_sampling, uniform_sampling, line_sampling
from utils_grid import norme
from constants import alpha, beta

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