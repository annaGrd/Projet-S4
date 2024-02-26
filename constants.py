import numpy as np

# Généralités
X = np.array([[0, 30], [0, 30], [0, 30]])  # faire choisir une taille de grille

Xobs = np.array([[[0, 25], [5, 10], [0, 30]], [[5, 30], [15, 20], [5, 30]]])  # à compléter
vObs = 0
l_min = 10  # longueur minimale d'un obstacle
k = 100 # nombre de points max dans la trajectoire

for obs in Xobs:
    x = obs[0][1] - obs[0][0]
    y = obs[1][1] - obs[1][0]
    z = obs[2][1] - obs[2][0]
    vObs += x*y*z

vTot = X[0][1] * X[1][1] * X[2][1]
vFree = vTot - vObs

# Algo 1
rprox = 5  # arbitraire, algo 1
edge = 2  # arbitraire, taille d'une case, choisir en fonction de la taille des objets dynamiques

# Algo 2
kmax = 5  # arbitraire
rs = 5  # arbitraire
alpha = .1  # arbitraire
beta = 2  # arbitraire

update_time = 10  # arbitraire, durée entre deux updates
safety_radius = 5  # arbitraire, rayon supplémentaire pour être sûr de prendre tous les noeuds dans la range d'un obstacle dynamique
ro = 10 # arbitraire
rg = 2 # arbitraire
