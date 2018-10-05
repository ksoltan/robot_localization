from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from math import cos, sin, radians

class Particle(object):
    def __init__(self, x, y, theta, weight):
        self._x = x
        self._y = y
        self._theta = theta
        self._weight = weight

    @property
    def pos(self):
        return (self._x, self._y, self._theta)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, new_x):
        self._x = new_x

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, new_y):
        self._y = new_y

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, new_theta):
        self._theta = new_theta

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, new_weight):
        self._weight = new_weight

    def make_copy(self):
        return Particle(x=self.x, y=self.y, theta=self.theta, weight=self.weight)

    def __str__(self):
        return "(Pos: {}: Weight: {})".format(self.pos, self.weight)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.theta == other.theta and self.weight == other.weight

    '''
    Function: get_marker
    Inputs:

    Return a Marker Arrow object. The origin of the arrow is the x, y position of
    the particle. The direction of the arrow is the angle (theta). The color of
    the arrow is proportional to the particle's weight.

    '''
    def get_marker(self):
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
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        # Euler to Quaternion angle
        marker.pose.orientation.w = cos(self.theta / 2.0)
        marker.pose.orientation.z = sin(self.theta / 2.0)
        marker.color = ColorRGBA(r=new_r, g=new_g, b=new_b, a=1)
        marker.scale = Vector3(0.2, 0.01, 0.01)

        return marker
