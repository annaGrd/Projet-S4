from time import time

from dynamic import update_goal_and_obstacles, update_block
from dynamic_obstacles import get_dynamic_obstacles
from tree import Tree
from constants import rprox, update_time
from utils_grid import norme

T = Tree()

def main(xa, Xobs):
    # Algo 1
    T.root = xa
    T.xgoal = xa
    T.Vt = [xa]
    T.add_node_to_cell(xa)

    beginExecutionTime = time()
    previousTraj = []

    while time() - beginExecutionTime < 60:  # à modifier en dynamique
        change_xgoal, dynamicObstacles = update_goal_and_obstacles(T, time()-beginExecutionTime)
        if change_xgoal:
            for x in T.Vt:
                x.already_seen = False

            T.update_root()

            T.traj = [T.root]  # On reset le chemin à chaque chanqement d'objectif

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
        # - Si il n'y a pas d'obstacles entre le drone et le prochain objectif, il peut y aller
        # - Si il y a un obstacle entre le drone et le prochain objectif, on actualise la racine et on recalcule la trajectoire
        toGo = None
        if len(T.traj) > 1 and moving:

            if not T.traj[1].line(T.xa):
                T.update_root()
                T.plan()

            toGo = T.traj[1]

        previousTraj = list(T.traj)

        T.nextNodeToGo = toGo