from time import time

import numpy as np

from dynamic import update
from tree import Tree
from noeud import Noeud
from constants import rprox, k
from utils_grid import norme


def main(xa, Xobs, xgoal):
    # Algo 1
    T = Tree([xa], xa, xgoal)

    # test
    listDronePositions = []
    listTraj = []

    while norme(T.xa, T.xgoal) > rprox:  # à modifier en dynamique
        print(norme(T.xa, T.xgoal))
        _, change_xgoal = False, False #update(T)
        if change_xgoal:
            for x in T.Vt:
                x.already_seen = False

        t = time()
        while time() - t < .5:  # Durée arbitraire
            T.expansion_and_rewiring()

        moving = T.plan()
        if len(T.traj) > 1 and norme(T.xa, T.traj[1]) <= rprox:  # Ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.root = T.traj[0]
            T.root.ci = 0
            T.traj[0].recalculate_child_costs()
            T.restart = True
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo si moving == True
        # code de test
        velocity = 1
        if len(T.traj) > 1 and moving:
            coordinatesXa = np.array([T.xa.x, T.xa.y, T.xa.z])
            coordinatesX1 = np.array([T.traj[1].x, T.traj[1].y, T.traj[1].z])
            coordinatesNewXa = (coordinatesX1 - coordinatesXa) / norme(T.xa, T.traj[1]) * velocity + coordinatesXa
            T.xa = Noeud(coordinatesNewXa[0], coordinatesNewXa[1], coordinatesNewXa[2])

        listDronePositions.append(T.xa)
        listTraj.append(T.traj)

    return listDronePositions, listTraj