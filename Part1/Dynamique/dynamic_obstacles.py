from time import time

"""
Le but ici est de simuler le fait que les informations arrivent en temps réel depuis l'extérieur
"""
t = time()

def get_dynamic_obstacles():
    relativeTime = time() - t
    x = 10 + 5 * relativeTime / 60
    y = 5 + 20 * relativeTime / 60

    return [(x, y, .5, 2)]