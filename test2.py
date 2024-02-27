from matplotlib import pyplot as plt, animation

from Programme import main
from constants import X, Xobs
from noeud import Noeud

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

xa = Noeud()

listDronePositions, listTraj = main(xa, [])

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

    dronePosition = listDronePositions[i]

    artists.append(ax.scatter(dronePosition.x, dronePosition.y, dronePosition.z))

    for nodeTrajIdx in range(len(listTraj[i])-1):
        artists.append(ax.plot([listTraj[i][nodeTrajIdx].x, listTraj[i][nodeTrajIdx+1].x], [listTraj[i][nodeTrajIdx].y, listTraj[i][nodeTrajIdx+1].y], [listTraj[i][nodeTrajIdx].z, listTraj[i][nodeTrajIdx+1].z], color="red"))

    return artists


anim = animation.FuncAnimation(fig, update_fig, len(listDronePositions))
plt.show()