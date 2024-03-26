from time import time
from copy import deepcopy

import numpy as np

from Part1.Dynamique.dynamic import update_goal_and_obstacles, update_block
from Part1.Dynamique.dynamic_obstacles import get_dynamic_obstacles
from Part1.MainAndClasses.tree import Tree
from Part1.MainAndClasses.noeud import Noeud
from Part1.MainAndClasses.constants import rprox, update_time
from Part1.Grille.utils_grid import norme

import os


def main(xa, Xobs):
    # Algo 1
    T = Tree([xa], xa)
    os.chdir("../..")
    with open("Passerelle1-2/m_to_py.csv", "w") as file:
        file.write(f"{0}\n{0}\n{0}\n{0}\n{0}\n{0}\n")

    # test
    listDronePositions = []
    listTraj = []
    listXgoal = []
    listDynamicObstacles = []
    listNodes = []
    listLinks = []

    beginExecutionTime = time()
    previousTraj = []

    while time() - beginExecutionTime < 300:  # à modifier en dynamique
        pos = open('Passerelle1-2/m_to_py.csv').read().split("\n")[:-1]
        while not pos:
            pos = open('Passerelle1-2/m_to_py.csv').read().split("\n")[:-1]
        x, y, z, _, _, _ = pos
        T.xa = Noeud(float(x), float(y), float(z))
        change_xgoal, dynamicObstacles = update_goal_and_obstacles(T, time()-beginExecutionTime)
        if change_xgoal:
            print("Changement d'objectif")
            for x in T.Vt:
                x.already_seen = False

            T.update_root()

            T.traj = [T.root]  # On reset le chemin à chaque changement d'objectif

        print("Temps actuel :", time() - beginExecutionTime, "Chemin trouvé : ", T.goal_reached()[0])
        if len(T.traj) > 1:
            print("Position : ", T.xa, "Prochain point : ", T.traj[1])
        print("Distance de l'objectif : " + str(norme(T.xa, T.xgoal)))
        print("Nombre de noeuds : " + str(len(T.Vt)))

        t = time()
        while time() - t < update_time:  # Durée arbitraire
            T.expansion_and_rewiring()

        dynamicObstacles = get_dynamic_obstacles()
        update_block(T, dynamicObstacles)

        moving = T.plan()

        if previousTraj != T.traj:
            T.update_root()
            moving = T.plan()

        if len(T.traj) > 1 and norme(T.xa, T.traj[1]) <= rprox:  # Ou faire avec le ci de traj[0]
            T.traj.pop(0)
            T.update_root(T.traj[0])
        # envoyer la traj au drone, il va vers xo si moving == True

        # Si jamais le drone s'est éloigné de la racine actuelle et que le chemin change, il y a 2 cas
        # - S'il n'y a pas d'obstacles entre le drone et le prochain objectif, il peut y aller
        # - S'il y a un obstacle entre le drone et le prochain objectif, on actualise la racine et on recalcule la trajectoire
        if len(T.traj) > 1 and moving:

            if not T.traj[1].line(T.xa):
                T.update_root()
                T.plan()

            # modif du csv
            with open("Passerelle1-2/py_to_m.csv", "w") as file:
                xValues = ",".join([str(node.x) for node in T.traj])
                yValues = ",".join([str(node.y) for node in T.traj])
                zValues = ",".join([str(node.z) for node in T.traj])
                psiValues = ",".join(["0" for _ in T.traj])
                file.write(f"{xValues}\n{yValues}\n{zValues}\n{psiValues}")

        previousTraj = list(T.traj)

        # code de test
        """velocity = 4
        if toGo is not None:
            coordinatesXa = np.array([T.xa.x, T.xa.y, T.xa.z])
            coordinatesX1 = np.array([toGo.x, toGo.y, toGo.z])
            if norme(T.xa, toGo) != 0:
                coordinatesNewXa = (coordinatesX1 - coordinatesXa) / norme(T.xa, toGo) * min(velocity, norme(T.xa, toGo)) + coordinatesXa
                T.xa = Noeud(coordinatesNewXa[0], coordinatesNewXa[1], coordinatesNewXa[2])"""



        listDronePositions.append(T.xa)
        listTraj.append(T.traj)
        listXgoal.append(T.xgoal)
        listDynamicObstacles.append(dynamicObstacles)
        listNodes.append(deepcopy(T.Vt))
        listLinks.append(deepcopy(T.Et))

        print("")

    return listDronePositions, listTraj, listXgoal, listDynamicObstacles, listNodes, listLinks