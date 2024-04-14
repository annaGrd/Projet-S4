import os

import numpy as np


def get_obstacles(file):
    os.chdir("../..")
    with open(file, "r") as f:
        lignes = [eval(x) for x in f.readlines()[6:-1] if x != "\n"]
        obstacles = np.array([[[obs[0][0], obs[-1][0]], [obs[0][1], obs[-1][1]], [obs[0][2], obs[-1][2]]] for obs in lignes])
    return obstacles