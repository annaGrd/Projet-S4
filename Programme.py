from time import time
from dynamic import update
from tree import Tree
from noeud import Noeud
from constants import rprox, k
from utils_grid import norme


def main(xa, Xobs, xgoal):
    # Algo 1
    T = Tree([xa], Noeud(), Noeud())
    t = time()

    while norme(T.xa, T.xgoal) > rprox:  # à modifier en dynamique
        _, change_xgoal = update(T)
        if change_xgoal:
            for x in T.Vt:
                x.already_seen = False
        while time() - t < 100:  # Durée arbitraire
            T.expansion_and_rewiring()

        T, moving = T.plan()
        if norme(T.xa, T.traj[0]) <= rprox:  # Ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.traj[0].recalculate_child_costs()
            T.restart = True
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo si moving == True