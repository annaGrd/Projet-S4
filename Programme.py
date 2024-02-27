from time import time

import numpy as np

from dynamic import update_goal_and_obstacles
from tree import Tree
from noeud import Noeud
from constants import rprox, k
from utils_grid import norme


def main(xa, Xobs):
    # Algo 1
    T = Tree([xa], xa)

    # test
    listDronePositions = []
    listTraj = []
    listXgoal = []

    beginExecutionTime = time()

    while time() - beginExecutionTime < 300:  # à modifier en dynamique
        change_xgoal = update_goal_and_obstacles(T, time()-beginExecutionTime)
        if change_xgoal:
            # print("Changement d'objectif")
            for x in T.Vt:
                x.already_seen = False

            potentialRoot = T.closest_node(T.xa)
            if potentialRoot is not None:
                T.root = potentialRoot  # Quand on change d'objectif, on prend comme racine le noeud le plus proche
                T.root.ci = 0
                T.root.recalculate_child_costs(change_of_root=True)
                T.Qs = list()

            T.traj = [T.root]  # On reset le chemin à chaque chanqement d'objectif

        # print("Distance de l'objectif : " + str(norme(T.xa, T.xgoal)))

        t = time()
        while time() - t < .5:  # Durée arbitraire
            T.expansion_and_rewiring()

        moving = T.plan()
        if len(T.traj) > 1 and norme(T.xa, T.traj[1]) <= rprox:  # Ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.root = T.traj[0]
            T.root.ci = 0
            T.root.recalculate_child_costs(change_of_root=True)
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo si moving == True
        # code de test
        velocity = 4
        if len(T.traj) > 1 and moving:
            coordinatesXa = np.array([T.xa.x, T.xa.y, T.xa.z])
            coordinatesX1 = np.array([T.traj[1].x, T.traj[1].y, T.traj[1].z])
            coordinatesNewXa = (coordinatesX1 - coordinatesXa) / norme(T.xa, T.traj[1]) * min(velocity, norme(T.xa, T.traj[1])) + coordinatesXa
            T.xa = Noeud(coordinatesNewXa[0], coordinatesNewXa[1], coordinatesNewXa[2])

        listDronePositions.append(T.xa)
        listTraj.append(T.traj)
        listXgoal.append(T.xgoal)

    return listDronePositions, listTraj, listXgoal
