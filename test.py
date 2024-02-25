from tree import Tree
from noeud import Noeud
from time import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from constants import Xobs
from utils_grid import norme

animate = True
render = True
if animate : render = False

fig = plt.figure()
ax = fig.add_subplot(projection="3d")


nodeListByFrame = []
linkListByFrame = []
QrByFrame = []
QsByFrame = []
endTrajByFrame = []


def update_fig(i):
    artists = []
    ax.clear()

    ax.set_xlim3d([0, 100])
    ax.set_ylim3d([0, 100])
    ax.set_zlim3d([0, 100])

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
        if node == xa:
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
xgoal = Noeud(50, 50, 0)

T = Tree([xa], xa, xgoal)
T.add_node_to_cell(xa)
T.traj = [xa]

t1 = time()
i = 0
while time() - t1 < .5:
    i += 1
    t = time()
    T.expansion_and_rewiring()
    print(i, time() - t)

    endTraj = T.closest_node(T.xgoal)
    print(endTraj.x, endTraj.y, endTraj.z, endTraj.ci)
    print(norme(xa, xgoal))
    print("")

    if animate or render:
        nodeListByFrame.append(list(T.Vt))
        linkListByFrame.append(list(T.Et))
        QsByFrame.append(list(T.Qs))
        QrByFrame.append(list(T.Qr))
        endTrajByFrame.append(endTraj)

if animate:
    anim = animation.FuncAnimation(fig, update_fig, i)
    plt.show()

if render:
    update_fig(i-1)