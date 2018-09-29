class Particle(object):
    def __init__(self, pos, weight):
        self._pos = pos
        self._weight = weight

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, new_pos):
        self._pos = new_pos

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, new_weight):
        self._weight = new_weight

    def __str__(self):
        return "(Pos: {}: Weight: {})".format(self._pos, self._weight)
