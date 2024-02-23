from noeud import Noeud

class Tree:
    def __init__(self, noeuds=None, xa = Noeud(), xgoal = Noeud()):
        if noeuds is None:
            noeuds = list()
        self.Et = list()
        self.Vt = noeuds
        self.Qs = list()
        self.Qr = list()
        self.traj = list()
        self.mem = list()
        self.xa = xa # position du drone
        self.xgoal = xgoal # faire choisir un point d'arriver