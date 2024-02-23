from math import inf, atan, cos, sin, pi
from time import time
from random import uniform
import numpy as np
from tree import Tree
from noeud import Noeud
from constants import Xobs, X, edge, rprox, kmax, rs, alpha, beta, vFree
from utils_grid import norme, inGrid, cells
from random_sampling import ellipse, line_sample, uniform_sampling

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


def randNode( T ):

    xgoal = T.xgoal
    xo = T.traj[0]
    Pr = uniform(0, 1)
    xclose = T.traj[-1]  # récupère le noeud le plus proche de xgoal pour le moment

    if Pr > 1 - alpha :

        root = np.array([xo.x, xo.y, xo.z])
        goal = np.array([xgoal.x, xgoal.y, xgoal.z])

        return line_sample(root, goal)

    elif Pr <= ( 1 - alpha ) / beta or xclose != xgoal :

        return uniform_sampling()

    else :
        cmin = norme( xo, xgoal)
        cbest = xgoal.ci # est-ce bien la distance entre xo et xgoal, pas sur
        a = cbest / 2
        b = ( cbest**2 + cmin**2 )**0.5 / 2

        root = np.array( [ xo.x, xo.y, xo.z ] )
        goal = np.array([xgoal.x, xgoal.y, xgoal.z])

        return ellipse(root, goal, a, b)

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