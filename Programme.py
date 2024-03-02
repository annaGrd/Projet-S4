from time import time
from copy import deepcopy

import numpy as np

from dynamic import update_goal_and_obstacles, update_block
from dynamic_obstacles import get_dynamic_obstacles
from tree import Tree
from noeud import Noeud
from constants import rprox, update_time
from utils_grid import norme


def update_root(T):
    potentialRoot = T.closest_node(T.xa)
    if potentialRoot is not None:
        T.root = potentialRoot  # Quand on change d'objectif, on prend comme racine le noeud le plus proche
        T.root.ci = 0
        if T.root.parent is not None:
            T.root.childs.append(T.root.parent)
            T.root.parent = None
        T.root.recalculate_child_costs(change_of_root=True)
        T.Qs = list()


def main(xa, Xobs):
    # Algo 1
    T = Tree([xa], xa)

    # test
    listDronePositions = []
    listTraj = []
    listXgoal = []
    listDynamicObstacles = []
    listNodes = []
    listLinks = []

    beginExecutionTime = time()
    previousTraj = []

    while time() - beginExecutionTime < 60:  # à modifier en dynamique
        change_xgoal, dynamicObstacles = update_goal_and_obstacles(T, time()-beginExecutionTime)
        if change_xgoal:
            print("Changement d'objectif")
            for x in T.Vt:
                x.already_seen = False

            update_root(T)

            T.traj = [T.root]  # On reset le chemin à chaque chanqement d'objectif

        print("Temps actuel :", time() - beginExecutionTime)
        print("Distance de l'objectif : " + str(norme(T.xa, T.xgoal)))
        print("Nombre de noeuds : " + str(len(T.Vt)))

        t = time()
        while time() - t < update_time:  # Durée arbitraire
            T.expansion_and_rewiring()

        dynamicObstacles = get_dynamic_obstacles()
        update_block(T, dynamicObstacles)

        moving = T.plan()

        if previousTraj != T.traj:
            update_root(T)
            moving = T.plan()

        if len(T.traj) > 1 and norme(T.xa, T.traj[1]) <= rprox:  # Ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.root = T.traj[0]
            T.root.ci = 0
            if T.root.parent is not None:
                T.root.childs.append(T.root.parent)
                T.root.parent = None

            T.root.recalculate_child_costs(change_of_root=True)
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo si moving == True

        # Si jamais le drone s'est éloigné de la racine actuelle et que le chemin change, il y a 2 cas
        # - Si il n'y a pas d'obstacles entre le drone et le prochain objectif, il peut y aller
        # - Si il y a un obstacle entre le drone et le prochain objectif, on actualise la racine et on recalcule la trajectoire
        toGo = None
        if len(T.traj) > 1 and moving:

            if not T.traj[1].line(T.xa):
                update_root(T)
                T.plan()

            toGo = T.traj[1]

        previousTraj = list(T.traj)

        # code de test
        velocity = 4
        if toGo is not None:
            coordinatesXa = np.array([T.xa.x, T.xa.y, T.xa.z])
            coordinatesX1 = np.array([toGo.x, toGo.y, toGo.z])
            if norme(T.xa, toGo) != 0:
                coordinatesNewXa = (coordinatesX1 - coordinatesXa) / norme(T.xa, toGo) * min(velocity, norme(T.xa, toGo)) + coordinatesXa
                T.xa = Noeud(coordinatesNewXa[0], coordinatesNewXa[1], coordinatesNewXa[2])

        listDronePositions.append(T.xa)
        listTraj.append(T.traj)
        listXgoal.append(T.xgoal)
        listDynamicObstacles.append(dynamicObstacles)
        listNodes.append(deepcopy(T.Vt))
        listLinks.append(deepcopy(T.Et))

        print("")

    return listDronePositions, listTraj, listXgoal, listDynamicObstacles, listNodes, listLinks
