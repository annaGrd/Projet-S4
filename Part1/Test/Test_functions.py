def check_loops(Vt):
    for x in Vt:
        pa = x.parent
        while pa is not None and pa != x:
            pa = pa.parent

        if pa is not None:
            return True

    return False


def check_tree_structure(Vt, root):
    parcours = [root]
    to_add = [1]
    while to_add:
        to_add = []
        for x in parcours:
            for c in x.childs:
                if c not in parcours:
                    to_add.append(c)

        parcours.extend(to_add)

    for x in Vt:
        if x not in parcours:
            return False
    return True