from tree import Tree
from noeud import Noeud
from time import time
import matplotlib.pyplot as plt

from utils_grid import norme

xa = Noeud()
xa.ci = 0
xgoal = Noeud(50, 50, 50)

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

print(len(T.Vt))

endTraj = T.closest_node(T.xgoal)
print(endTraj.x, endTraj.y, endTraj.z, endTraj.ci)
print(norme(xa, xgoal))

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
for node in T.Vt:
    color = "black"
    if node in T.Qs:
        color = "blue"
    if node in T.Qr:
        color = "yellow"
    if node == xa:
        color = "red"
    if node == endTraj:
        color = "green"
    ax.scatter(node.x, node.y, node.z, color=color)

ax.scatter(xgoal.x, xgoal.y, xgoal.z, color="purple")

for node1, node2 in T.Et:
    ax.plot([node1.x, node2.x], [node1.y, node2.y], [node1.z, node2.z])

plt.show()