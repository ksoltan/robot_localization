from geometry_msgs.msg import Pose
from math import cos, sin, radians

class Particle(object):
    def __init__(self, x, y, theta, weight=0):
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
        return abs(self.x - other.x) < 0.0001 and \
                abs(self.y - other.y) < 0.0001 and \
                abs(self.theta - other.theta) < 0.0001 and \
                abs(self.weight - other.weight) < 0.0001

    '''
    Function: get_pose
    Inputs:

    Return the Pose of the particle.

    '''
    def get_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        # Euler to Quaternion angle
        pose.orientation.w = cos(radians(self.theta) / 2.0)
        pose.orientation.z = sin(radians(self.theta) / 2.0)

        return pose
