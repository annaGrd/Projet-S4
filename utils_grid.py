from constants import Xobs, edge, X


def norme(x1, x2):
    x = x1.x - x2.x
    y = x1.y - x2.y
    z = x1.z - x2.z
    return (x**2 + y**2 + z**2)**0.5


def inGrid(n):
    for obs in Xobs:
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


def cells():
    grid = []
    lx = X[0][1]  # lg selon x
    ly = X[1][1]  # lg selon y
    lz = X[2][1]  # lg selon z
    qx, rx = divmod(lx, edge)
    qy, ry = divmod(ly, edge)
    qz, rz = divmod(lz, edge)
    coordinatesx = [[i*edge, (i+1)*edge] for i in range(qx)]
    coordinatesy = [[i * edge, (i + 1) * edge] for i in range(qy)]
    coordinatesz = [[i * edge, (i + 1) * edge] for i in range(qz)]
    if rx: coordinatesx += [[qx * edge, qx * edge + rx]]
    if ry: coordinatesy += [[qy * edge, qy * edge + ry]]
    if rz: coordinatesz += [[qz * edge, qz * edge + rz]]
    for cx in coordinatesx:
        for cy in coordinatesy:
            for cz in coordinatesz:
                grid.append([cx, cy, cz, list()])
    return grid

def mkeXsi(x, cell):

    Xsi = list()
    qx = int(x.x // edge)
    qy = int(x.y // edge)
    qz = int(x.z // edge)
    nodes = cell[qx][qy][qz][3]

    for e in nodes:
        if e != x:
            Xsi.append(e)
