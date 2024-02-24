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
    lx = X[0][1] - X[0][0]  # lg selon x
    ly = X[1][1] - X[1][0]  # lg selon y
    lz = X[2][1] - X[2][0]  # lg selon z

    nbCellsx, rx = divmod(lx, edge)
    nbCellsy, ry = divmod(ly, edge)
    nbCellsz, rz = divmod(lz, edge)
    if rx: nbCellsx += 1
    if ry: nbCellsy += 1
    if rz: nbCellsz += 1

    return [[[[] for _ in range(nbCellsz)] for _ in range(nbCellsy)] for _ in range(nbCellsx)]
