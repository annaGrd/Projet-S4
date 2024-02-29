from time import time

"""
Le but ici est de simuler le fait que les informations arrivent en temps réel depuis l'extérieur
"""
global t
t = time()

def get_dynamic_obstacles():

    if time()-t < 5:
        return [(15, 25, 15, 2)]
    else:
        return [(15, 20, 15, 2)]