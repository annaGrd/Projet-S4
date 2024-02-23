import numpy as np

# Généralités
X = np.array([[0, 1000], [0, 1000], [0, 1000]])  # faire choisir une taille de grille

Xobs = np.array([])  # à compléter
vObs = 0

for obs in Xobs:
    x = obs[0][1] - obs[0][0]
    y = obs[1][1] - obs[1][0]
    z = obs[2][1] - obs[2][0]
    vObs += x*y*z

vTot = X[0][1] * X[1][1] * X[2][1]
vFree = vTot - vObs

# Algo 1
rprox = 5  # arbitraire, algo 1
edge = 10  # arbitraire, taille d'une case, choisir en fonction de la taille des objets dynamiques

# Algo 2
kmax = 5  # arbitraire
rs = 0.5  # arbitraire
alpha = 0.1  # arbitraire
beta = 2  # arbitraire
