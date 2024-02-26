from tree import Tree
from noeud import Noeud
from time import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from constants import Xobs, X
from utils_grid import norme

animate = False  # Anime tout le procede de l'algo
render = True  # Affiche les obstacles, la trajectoire trouvee le point de depart et le point d'arrivee

fig = plt.figure()
ax = fig.add_subplot(projection="3d")


nodeListByFrame = []
linkListByFrame = []
QrByFrame = []
QsByFrame = []
endTrajByFrame = []
xaByFrame = []


def update_fig(i):
    artists = []
    ax.clear()

    ax.set_xlim3d([X[0][0], X[0][1]])
    ax.set_ylim3d([X[1][0], X[1][1]])
    ax.set_zlim3d([X[2][0], X[2][1]])

    for xobs in Xobs:
        for x in [[xobs[0][0], xobs[0][0]], [xobs[0][1], xobs[0][0]], [xobs[0][1], xobs[0][1]]]:
            for y in [[xobs[1][0], xobs[1][0]], [xobs[1][1], xobs[1][0]], [xobs[1][1], xobs[1][1]]]:
                for z in [[xobs[2][0], xobs[2][0]], [xobs[2][1], xobs[2][0]], [xobs[2][1], xobs[2][1]]]:
                    artists.append(ax.plot(x, y, z, color="red"))


    for node in nodeListByFrame[i]:
        color = "black"
        if node in QsByFrame[i]:
            color = "blue"
        if node in QrByFrame[i]:
            color = "yellow"
        if node == xaByFrame[i]:
            color = "red"
        if node == endTrajByFrame[i]:
            color = "green"
        artists.append(ax.scatter(node.x, node.y, node.z, color=color))

    artists.append(ax.scatter(xgoal.x, xgoal.y, xgoal.z, color="purple"))

    for node1, node2 in linkListByFrame[i]:
        artists.append(ax.plot([node1.x, node2.x], [node1.y, node2.y], [node1.z, node2.z], color="blue"))

    if animate:
        return artists
    plt.show()


xa = Noeud()
xa.ci = 0
xgoal = Noeud(30, 30, 20)

T = Tree([xa], xa, xgoal)
T.add_node_to_cell(xa)
T.traj = [xa]

timeSpent = 0
i = 0
while timeSpent < 5:
    i += 1

    """if time() - t1 > .25:
        xa = T.Vt[17]
        T.xa = xa
        xa.ci = 0
        xa.recalculate_child_costs()"""

    t = time()
    T.expansion_and_rewiring()
    timeSpent += time() - t
    print(i, time() - t)

    endTraj = T.closest_node(T.xgoal)
    print(endTraj.x, endTraj.y, endTraj.z, endTraj.ci, norme(endTraj, xgoal))
    print(norme(xa, xgoal))
    print(len(T.Vt))
    print("")

    if animate or render:
        nodeListByFrame.append(list(T.Vt))
        linkListByFrame.append(list(T.Et))
        QsByFrame.append(list(T.Qs))
        QrByFrame.append(list(T.Qr))
        endTrajByFrame.append(endTraj)
        xaByFrame.append(xa)

if animate:
    anim = animation.FuncAnimation(fig, update_fig, i)
    plt.show()

if render:

    ax.set_xlim3d([X[0][0], X[0][1]])
    ax.set_ylim3d([X[1][0], X[1][1]])
    ax.set_zlim3d([X[2][0], X[2][1]])

    endTrajPath = endTrajByFrame[-1]
    ax.scatter(endTrajPath.x, endTrajPath.y, endTrajPath.z, color="green")
    while endTrajPath.parent(xa) is not None:
        pa = endTrajPath.parent(xa)
        ax.scatter(pa.x, pa.y, pa.z, color="blue")
        ax.plot([endTrajPath.x, pa.x], [endTrajPath.y, pa.y], [endTrajPath.z, pa.z], color="blue")
        endTrajPath = pa

    for xobs in Xobs:
        for x in [[xobs[0][0], xobs[0][0]], [xobs[0][1], xobs[0][0]], [xobs[0][1], xobs[0][1]]]:
            for y in [[xobs[1][0], xobs[1][0]], [xobs[1][1], xobs[1][0]], [xobs[1][1], xobs[1][1]]]:
                for z in [[xobs[2][0], xobs[2][0]], [xobs[2][1], xobs[2][0]], [xobs[2][1], xobs[2][1]]]:
                    ax.plot(x, y, z, color="red")

    ax.scatter(xgoal.x, xgoal.y, xgoal.z, color="purple")

    plt.show()