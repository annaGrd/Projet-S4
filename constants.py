import numpy as np

# Généralités
X = np.array([[0, 30], [0, 30], [0, 2]])  # taille de grille

Xobs = np.array([[[0, 25], [8, 10], [0, 2]], [[5, 30], [18, 20], [0, 2]]])  # liste des obstacles
vObs = 0  # Volume occupé par les obstacles
k = 100 # nombre de points max dans la trajectoire

for obs in Xobs:
    x = obs[0][1] - obs[0][0]
    y = obs[1][1] - obs[1][0]
    z = obs[2][1] - obs[2][0]
    vObs += x*y*z

vTot = X[0][1] * X[1][1] * X[2][1]
vFree = vTot - vObs

rprox = .5  # Algo 1
edge = 2  # Taille d'une case, à définir en fonction de la taille des objets dynamiques

kmax = 5  # Nombre maximal de voisins
rs = 3  # Distance minimale entre deux noeuds
alpha = .1  # Constante pour le random sampling
beta = 2  # Constante pour le random sampling

update_time = .5  # Durée entre deux updates
safety_radius = .5  # Rayon supplémentaire pour être sûr de prendre tous les noeuds dans la range d'un obstacle dynamique
ro = 100  # Rayon dans lequel les obstacles dynamiques sont considérés par le drone (marqueur block)
rg = 3  # Distance en dessous de laquelle, un nœud atteint xgoal