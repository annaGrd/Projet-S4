from matplotlib import pyplot as plt, animation
import numpy as np

from Part1.MainAndClasses.Programme import main
from Part1.MainAndClasses.constants import X, Xobs
from Part1.MainAndClasses.noeud import Noeud

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

xa = Noeud(15, 15, 15)

listDronePositions, listTraj, listXgoal, listDynamicObstacles, listNodes, listLinks = main(xa, [])


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
                    artists.append(ax.plot(x, y, z, color="black"))

    for dobs in listDynamicObstacles[i]:
        robs = dobs[3]
        # Merci stackoverflow mdr
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        x = dobs[0] + robs*np.cos(u) * np.sin(v)
        y = dobs[1] + robs*np.sin(u) * np.sin(v)
        z = dobs[2] + robs*np.cos(v)
        ax.plot_surface(x, y, z)

    dronePosition = listDronePositions[i]

    artists.append(ax.scatter(dronePosition.x, dronePosition.y, dronePosition.z))

    for nodeTrajIdx in range(len(listTraj[i])-1):
        artists.append(ax.plot([listTraj[i][nodeTrajIdx].x, listTraj[i][nodeTrajIdx+1].x], [listTraj[i][nodeTrajIdx].y, listTraj[i][nodeTrajIdx+1].y], [listTraj[i][nodeTrajIdx].z, listTraj[i][nodeTrajIdx+1].z], color="red"))

    artists.append(ax.scatter(listXgoal[i].x, listXgoal[i].y, listXgoal[i].z, color="purple"))

    return artists


anim = animation.FuncAnimation(fig, update_fig, len(listDronePositions))
anim.save("test.gif")