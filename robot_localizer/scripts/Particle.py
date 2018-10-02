from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from math import cos, sin, radians

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
    def x(self):
        return self._pos[0]

    @x.setter
    def x(self, new_x):
        self._pos[0] = new_x

    @property
    def y(self):
        return self._pos[1]

    @y.setter
    def y(self, new_y):
        self._pos[1] = new_y

    @property
    def theta(self):
        return self._pos[2]

    @theta.setter
    def theta(self, new_theta):
        self._pos[2] = new_theta

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, new_weight):
        self._weight = new_weight

    def __str__(self):
        return "(Pos: {}: Weight: {})".format(self._pos, self._weight)

    '''
    Function: get_marker
    Inputs:

    Return a Marker Arrow object. The origin of the arrow is the x, y position of
    the particle. The direction of the arrow is the angle (theta). The magnitude
    and color of the arrow are proportional to the particle's weight.

    '''
    def get_marker(self):
        # Calculate end point of the arrow
        max_magnitude = 1
        end_pt_x = self.x + cos(radians(self.theta)) * (self._weight * max_magnitude)
        end_pt_y = self.y + sin(radians(self.theta)) * (self._weight * max_magnitude)

        # Calculate color. Blue represents likelihood of 1. Red is likelihood 0.
        min_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1)
        max_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1)
        new_r = self._weight * max_color.r + (1 - self._weight) * min_color.r
        new_g = self._weight * max_color.g + (1 - self._weight) * min_color.g
        new_b = self._weight * max_color.b + (1 - self._weight) * min_color.b

        # Create a marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.ARROW
        marker.points = [Vector3(self.x, self.y, 0), Vector3(end_pt_x, end_pt_y, 0)]
        marker.color = ColorRGBA(r=new_r, g=new_g, b=new_b, a=1)
        marker.scale = Vector3(1, 1, 1)
        print("Added marker: x, y: {}, {}; theta: {}".format(self.x, self.y, self.theta))
        return marker
