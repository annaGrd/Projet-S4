from time import time

"""
Le but ici est de simuler le fait que les informations arrivent en temps réel depuis l'extérieur
"""
t = time()

def get_dynamic_obstacles():
    relative_time = time() - t
    y = 20 + 5*relative_time/10

    return [(15, y, 15, 1)]
