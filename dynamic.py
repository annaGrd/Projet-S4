from tree import Tree
from noeud import Noeud
from constants import update_time, safety_radius


"""obstacles est une liste contenant les obstacles dynamiques de la forme [x, y, z, vitesse]"""
def update_block(T, obstacles):

    # on récupère tous les noeuds marqués
    blocked = list()
    for x in T.Vt:
        if x.block:
            blocked.append(x)

    # on récupère tous les noeuds dans la range rb de chaque obstacle
    for obs in obstacles:
        x_inrange = calcul_inrange(obs)

        # si le nœud est déjà marqué, il le restera, sinon, il le devient
        for x in x_inrange:
            if x.block: blocked.remove(x)
            x.block = True

    # il ne reste plus que les noeuds qui ne sont plus en range d'un obstacle
    for x in blocked:
        x.block = False


def calcul_inrange(obs):
    rb = update_time * obs[3] + safety_radius