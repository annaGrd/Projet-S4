from tree import Tree
from noeud import Noeud
from time import time
import matplotlib.pyplot as plt

xa = Noeud()
xa.ci = 0
xgoal = Noeud(50, 50, 25)

T = Tree([xa], xa, xgoal)
T.add_node_to_cell(xa)
T.traj = [xa]

for _ in range(25):
    t = time()
    T.expansion_and_rewiring()
    print(time() - t)

endTraj = T.closest_node(T.xgoal)
print(endTraj.x, endTraj.y, endTraj.z, endTraj.ci)

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
for node in T.Vt:
    color = "red"
    if node in T.Qs:
        color = "green"
    if node in T.Qr:
        color = "blue"
    if node == xa:
        color = "black"
    ax.scatter(node.x, node.y, node.z, color=color)

for node1, node2 in T.Et:
    ax.plot([node1.x, node2.x], [node1.y, node2.y], [node1.z, node2.z])

plt.show()