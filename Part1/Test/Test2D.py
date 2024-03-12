from matplotlib import pyplot as plt, animation
import numpy as np

from Part1.MainAndClasses.Programme import main
from Part1.MainAndClasses.constants import X, Xobs
from Part1.MainAndClasses.noeud import Noeud

fig = plt.figure()
ax = fig.add_subplot()

xa = Noeud()

listDronePositions, listTraj, listXgoal, listDynamicObstacles, listNodes, listLinks = main(xa, [])

def update_fig(i):
    artists = []
    ax.clear()

    ax.set_xlim([X[0][0], X[0][1]])
    ax.set_ylim([X[1][0], X[1][1]])

    for xobs in Xobs:
        for x in [[xobs[0][0], xobs[0][0]], [xobs[0][1], xobs[0][0]], [xobs[0][1], xobs[0][1]]]:
            for y in [[xobs[1][0], xobs[1][0]], [xobs[1][1], xobs[1][0]], [xobs[1][1], xobs[1][1]]]:
                artists.append(ax.plot(x, y, color="black"))

    for dobs in listDynamicObstacles[i]:
        robs = dobs[3]
        u = np.linspace(0, 2*np.pi)
        x = dobs[0] + robs*np.cos(u)
        y = dobs[1] + robs*np.sin(u)
        artists.append(ax.plot(x, y))

    for node in listNodes[i]:
        if node.ci == float("inf"):
            color = "yellow"
        else:
            v = min(1., node.ci/100)
            color = (v, v, v)
        artists.append(ax.scatter(node.x, node.y, color=color))

    for link in listLinks[i]:
        color = "black"
        artists.append(ax.plot([link[0].x, link[1].x], [link[0].y, link[1].y], color=color))

    dronePosition = listDronePositions[i]

    artists.append(ax.scatter(dronePosition.x, dronePosition.y))

    for nodeTrajIdx in range(len(listTraj[i])-1):
        artists.append(ax.plot([listTraj[i][nodeTrajIdx].x, listTraj[i][nodeTrajIdx+1].x], [listTraj[i][nodeTrajIdx].y, listTraj[i][nodeTrajIdx+1].y], color="red"))

    artists.append(ax.scatter(listXgoal[i].x, listXgoal[i].y, color="purple"))

    return artists


anim = animation.FuncAnimation(fig, update_fig, len(listDronePositions))
anim.save("test.gif")