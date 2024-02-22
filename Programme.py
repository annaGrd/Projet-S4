from math import inf, atan, cos, sin, pi
from time import time
from random import uniform
import numpy as np

class Noeud:
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x = x
        self.y = y
        self.z = z
        self.block = False # être exploré ou non
        self.ci = inf
        self.voisins = [ np.array( list() ), np.array( list() ) ] # fusionner

    def __eq__( self, other ):
        if self.x == other.x and self.y == other.y and self.z == other.z :
            return True
        else:
            return False


class Tree:
    def __init__(self, noeuds=None, xa = Noeud(), xgoal = Noeud()):
        if noeuds is None:
            noeuds = list()
        self.Et = list()
        self.Vt = noeuds
        self.Qs = list()
        self.Qr = list()
        self.traj = list()
        self.mem = list()
        self.xa = xa # position du drone
        self.xgoal = xgoal # faire choisir un point d'arriver

X = np.array([[0,1000],[0,1000],[0,1000]]) # faire choisir une taille de grille

Xobs = np.array([]) # à compléter
vObs = 0

for obs in Xobs :

    x = obs[0][1] - obs[0][0]
    y = obs[1][1] - obs[1][0]
    z = obs[2][1] - obs[2][0]
    vObs += x*y*z

vTot = X[0][1] * X[1][1] * X[2][1]
vFree = vTot - vObs


rprox = 5 # arbitraire, algo 1

def norme(x1,x2):
    x = x1.x - x2.x
    y = x1.y - x2.y
    z = x1.z - x2.z
    return (x**2 + y**2 + z**2)**0.5

def inGrid( n ) :
    for obs in Xobs :
        if (obs[0][0] >= n.x or n.x >= obs[0][1]) or (obs[1][0] >= n.y or n.y >= obs[1][1]) or (
                obs[2][0] >= n.z or n.z >= obs[2][1]):
            return False
        else:
            pass
    return True

"""            break
    else:
        return True
    return False """

edge = 10 # arbitraire, taille d'une case, choisir en fonction de la taille des objets dynamiques

def cells():
    grid = np.array([])
    lx = X[0][1] # lg selon x
    ly = X[1][1] # lg selon y
    lz = X[2][1] # lg selon z
    qx, rx = divmod( lx, edge )
    qy, ry = divmod( ly, edge )
    qz, rz = divmod( lz, edge )
    for i in range( qx-1 ):
        for j in range( qy -1 ):
            for k in range( qz -1 ):
                np.append(grid, [[ i*edge, (i+1)*edge ], [ j*edge, (j+1)* edge ], [ k*edge, (k+1)*edge ], list() ])

            np.append(grid, [[ i*edge, (i+1)*edge ], [ j*edge, (j+1)* edge ], [ qz*edge, qz*edge + rz ], list() ])

        np.append(grid, [[ i*edge, (i+1)*edge ], [ qy*edge, qy*edge + ry ], [ qz*edge, qz*edge + rz ], list() ])

    np.append(grid, [[ qx*edge, qx*edge + rx ], [ qy*edge, qy*edge + ry ], [ qz*edge, qz*edge + rz ], list() ])
    return grid

cell = cells()

def mkeXsi( x ) :

    Xsi = list()
    qx = x.x // edge
    qy = x.y // edge
    qz = x.z // edge
    l = cell[qx][qy][qz][3]

    for e in l :
        if e != x :
            Xsi.append(e)



## Algo 1

def main(xa, Xobs, xgoal):

    T = Tree( [xa], Noeud(), Noeud() ) # définir xa et xgoal
    t = time()

    while norme( T.xa, T.xgoal ) > rprox : # ? le drone proche de xgoal, à modifier en dynamique
    # On récupère le xa du drone et l'évolution de l'environnement
        while time() - t < 100: #duree arbitraire
            T = expAndRew( T )

        T = plan( T )
        if norme( T.xa, T.traj[0] ) <= rprox : # ou faire avec le ci de traj[0]

            T.traj.pop(0)
            T.Qs = list()
        # envoyer la traj au drone, il va vers xo
    return #?

## Algo 2
kmax  = 5 # arbritraire
rs = 0.5 # arbitraire

alpha = 0.1 # arbitraire
beta = 2 # arbitraire

def anglePhi( vect ):

    if vect[0] != 0 and vect[1] != 0:

        return atan( vect[2] / (( vect[0]**2 + vect[1]**2 )**0.5 ))

    else :

        return 0

def rotaPhi( phi, vect ): #phi angle de latitude

    mat = np.array( [ [1, 0, 0 ], [ 0, sin(phi), -cos(phi) ], [ 0, cos(phi), sin(phi)] ] )

    return vect*mat

def angleTheta( vect ):

    if vect[0] != 0 and vect[1] != 0:

        return 2*atan( vect[1] / ( vect[0] + ( vect[0]**2 + vect[1]**2 )**0.5 ))

    else:

        return 0

def rotaTheta( theta, vect ):

    mat = np.array( [[ sin(theta), -cos(theta), 0 ], [ cos(theta), sin(theta), 0 ], [ 0, 0, 1 ]] )

    return vect*mat


def ellipsoid( x, y, z, a, b ):

    return ( x**2 + y**2) / b**2 + z**2 / a**2 <= 1


def ellipse( root, mid, a, b ):

    # translation
    root -= mid

    # rotation phi
    phi = anglePhi( root )

    # rotation theta
    theta = angleTheta( root )

    # recherche coordonnées, on peut le faire dès le début non ?
    x = uniform(-b, b )
    y = uniform(-b, b )
    z = uniform(-a, a )
    "c'est la merde, comment je teste que c'est dans la grille ?. Pour le moment, je le fais plus tard mais un peu contre-productif."
    while not ellipsoid(x, y, z, a, b):
        x = uniform(-b, b )
        y = uniform(-b, b )
        z = uniform(-a, a )

    new = np.array( [x, y, z] )

    new = rotaTheta( theta, new )
    new = rotaPhi( phi, new )
    new += mid

    # moyen de faire plus joli que de réécrire la même chose dans le while ?
    while not inGrid( Noeud(new[0], new[1], new[2]) ) :
        x = uniform(-b, b )
        y = uniform(-b, b )
        z = uniform(-a, a )

        while not ellipsoid(x, y, z, a, b):
            x = uniform(-b, b )
            y = uniform(-b, b )
            z = uniform(-a, a )

        new = root + np.array( [x, y, z] )

        new = rotaTheta( theta, new )
        new = rotaPhi( phi, new )
        new += mid

    return new[0], new[1], new[2]


def randNode( T ):

    xgoal = T.xgoal
    Pr = uniform()
    xclose = T.traj[-1]  # récupère le noeud le plus proche de xgoal pour le moment

    if Pr > 1 - alpha :

        r = uniform()
        x = xgoal.x - xclose.x
        y = xgoal.y - xclose.y
        z = xgoal.z - xclose.z

        while not inGrid(Noeud(r*x + xclose.x, r*y + xclose.y, r*z + xclose.z)):
            r = uniform()

        return Noeud( r*x + xclose.x, r*y + xclose.y, r*z + xclose.z)

    elif Pr <= ( 1 - alpha ) / beta or xclose != xgoal :

        x = uniform( X[0][0], X[0][1] )
        y = uniform( X[1][0], X[1][1] )
        z = uniform( X[2][0], X[2][1] )

        while not inGrid(Noeud(x,y,z)):

            x = uniform( X[0][0], X[0][1] )
            y = uniform( X[1][0], X[1][1] )
            z = uniform( X[2][0], X[2][1] )

        return Noeud(x,y,z)

    else :

        xo = T.traj[0]
        cmin = norme( xo, xgoal)
        cbest = xgoal.ci # est-ce bien la distance entre xo et xgoal, pas sur
        a = cbest / 2
        b = ( cbest**2 + cmin**2 )**0.5 / 2

        mid = np.array([(xo.x + xgoal.x)/2, (xo.y + xgoal.y)/2,(xo.z + xgoal.z)/2])
        root = np.array( [ xo.x, xo.y, xo.z ] )

        x, y, z = ellipse( root, mid, a, b)

        return Noeud(x, y, z)

def findNodesNear( T, x, Xsi):

    Xnear = list()
    epsilon = ( ( vFree * kmax ) / (pi * len( T.Vt ) ) )**0.5

    if epsilon < rs :
        epsilon = rs

    for xnear in Xsi :
        if norme( x, xnear) < epsilon :
            Xnear.append( xnear )

    return Xnear

def expAndRew( T ):

    xrand = randNode( T )
    Xsi = # à faire

    dmin = min([ norme( xrand, xi) for xi in Xsi]) # surement plus optimisé à faire
    rang = Xsi.index( dmin )
    xclosest = Xsi[rang] # cette ligne + les deux au-dessus pour remplacer argmin

    if line( xclosest, xrand ): # n'existe pas encore
        Xnear = findNodesNear( T, xrand, Xsi )

        if len( Xnear ) < kmax or norme( xclosest, xrand ) > rs :
            addNode( T, xrand, xclosest, Xnear)
            T.Qr.insert( xrand, 0)

        else:
            T.Qr.insert( xclosest, 0)

        T = rewireRandNode( T )
    T = rewireRoot( T )

    return T

## Algo 3

def addNode( T, xnew, xclosest, Xnear):

    xmin = xclosest
    cmin = cost( xclosest ) + norme( xclosest, xnew )

    for xnear in Xnear :

        cnew = cost( xnear ) + norme( xnear, xnew )

        if cnew < cmin and line( xnear, xnew ) :
            cmin = cnew
            xmin = xnear

    T.Vt.append( xnew )
    T.Et.append( (xmin, xnew ) ) # faire un dictionnaire
    xnew.voisins[0].append( xmin )
    xnew.voisins[1].append( xmin.ci )
    xmin.voisins[0].append( xnew )
    xmin.voisins[1].append( xnew.ci )

    # Ajout du noeud dans la case correspondante dans la grille
    qx = xnew.x // edge
    qy = xnew.y // edge
    qz = xnew.z // edge
    cell[qx][qy][qz][3].append( xnew )

    return None


## Algo 4

def fc( T, xc ):

    # à modifier si on passe à un xgoal dynamique
    if xc.marqueur :
        return inf

    else:
        return cost( xc ) + norme( xc, T.xgoal )


def parent( T, voisins, ci ):
    # prendre celui avec le ci plus petit que x

    for i in range( len(ci) ):
        ci[i] = fc( T, voisins[i] )

    cmin = ci[0]
    pa = voisins[0]

    for voisin in voisins:

        if voisin[1] < cmin :

            cmin = voisin[1]
            pa = voisin[0]

    return [ pa, cmin ] # retourne un tuple car pratique pour bestChild
# mettre en commun des bouts de algo 4 et 5
def rewireRandNode( T ):

    t = time()
    while t - time() < 100 or len( T.Qr ) > 0: # Temps arbitraire

        xr = T.Qr.pop(0)
        Xsi = # n'existe pas encore
        Xnear = findNodesNear( T, xr, Xsi )

        for xnear in Xnear :
            cold = cost( xnear )
            cnew = cost( xr ) + norme( xr, xnear )

            if cnew < cold and line( xr, xnear ):
                T.Et.append( (xr, xnear) )
                pa = parent( T, xnear.voisins[0], xnear.voisins[1] )[0]

                if (xnear, pa) in T.Et :
                    T.Et.remove( (xnear, pa) )
                else: # volontairement un else pour déclencher une erreur si cette arête n'existait pas
                    T.Et.remove( (pa, xnear) )

                if xnear not in T.Qr : # cette condition est censée être toujours vraie
                    T.Qr.append( xnear )
                xr.voisins[0].append( xnear )
                xr.voisins[1].append( cnew )
    return T

## Algo 5

def rewireRoot( T ):

    if len( T.Qs ) == 0:
        T.Qs.append( T.traj[0] )
        T.mem = list()

    t = time()
    while t - time() < 100 or len( T.Qs ) > 0:

        xs = T.Qs.pop(0)
        Xnear = findNodesNear( T, xs, Xsi )

        for xnear in Xnear :

            cold = cost( xnear )
            cnew = cost( xs ) + norme( xs, xnear )

            if cnew < cold and line( xs, xnear ):

                T.Et.append( (xs, xnear) )
                pa = parent( T, xnear.voisins[0], xnear.voisins[1] )

                if (xnear, pa) in T.Et :
                    T.Et.remove( (xnear, pa) )

                else:
                    T.Et.remove( (pa, xnear) )

                xs.voisins[0].append( xnear )
                xs.voisins[1].append( cnew )

            if xnear not in T.mem : # à ajuster quand on gérera les obstacles dynamiques
                T.Qs.append( xnear )
                T.mem.append( xnear )

    return T

## Algo 6

def bestChild( T, x ):

    v = x.voisins[0]
    c = x.voisins[1]

    rg = c.index(parent( T, x.voisins[0], x.voisins[1]) )
    v.pop( rg ) # retire le parent
    c.pop( rg ) # et son coût

    return parent(T, v, c) # on récupère donc le deuxième meilleur voisin

def deadEnd( T, x ):
    if len( x.voisins[0] ) < 2 : # si n'a qu'un voisin ( juste son parent )
        return True

    else :
        pa = parent( T, x, x.ci )
        enfant = x.voisins[0] # référence partagée ?
        enfant.remove( pa ) # obtient liste des noeuds enfants

        for v in enfant :
            if not v.ci == inf : # si un enfant n'est pas marqué
                return False

        return True # tous les enfants sont marqués

def plan( T ):

    if norme( T.traj[-1], T.xgoal) < rprox :
        T.traj[-1] = T.xgoal
        # chemin toujours accessible ? check les ci et compare les arêtes

    else:
        path = [ T.traj[0] ]
        while deadEnd( T, path[-1] ) :

            path.append( T, bestChild( path[-1] ) )

        path[-1].block = True
        if fc( path[-1] ) == fc( T, T.traj[-1] ):

        # comparer les arêtes, voir si elles ont changé. regarder les ci, voir si l'un est infini
        """faire une diff de norme entre xa,xgoal et xk,goal"""

    return T