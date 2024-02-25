from constants import Xobs, edge, X


def norme(x1, x2):
    x = x1.x - x2.x
    y = x1.y - x2.y
    z = x1.z - x2.z
    return (x**2 + y**2 + z**2)**0.5


def inGrid(n):

    if n.x > X[0][1] or n.y > X[1][1] or n.z > X[2][1] or n.x < X[0][0] or n.y < X[1][0] or n.z < X[2][0]:
        return False

    for obs in Xobs:
        if (obs[0][0] <= n.x <= obs[0][1]) and (obs[1][0] <= n.y <= obs[1][1]) and (
                obs[2][0] <= n.z <= obs[2][1]):
            return False
    return True


"""            break
    else:
        return True
    return False """


def cells():
    lx = X[0][1]  # lg selon x
    ly = X[1][1]  # lg selon y
    lz = X[2][1]  # lg selon z

    nbCellsx, rx = divmod(lx, edge)
    nbCellsy, ry = divmod(ly, edge)
    nbCellsz, rz = divmod(lz, edge)
    if rx: nbCellsx += 1
    if ry: nbCellsy += 1
    if rz: nbCellsz += 1

    return [[[[] for _ in range(int(nbCellsz))] for _ in range(int(nbCellsy))] for _ in range(int(nbCellsx))]


def list_indices_at_range(r):
    listIndices = []

    for x in (r, -r):
        for y in range(-r, r + 1):
            for z in range(-r, r + 1):
                listIndices.append((x, y, z))

    for x in range(-r + 1, r):
        for y in (-r, r):
            for z in range(-r, r + 1):
                listIndices.append((x, y, z))

    for x in range(-r + 1, r):
        for y in range(-r + 1, r):
            for z in (-r, r):
                listIndices.append((x, y, z))

    return listIndices