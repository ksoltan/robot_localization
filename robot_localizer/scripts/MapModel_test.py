import unittest
from MapModel import *
import random
from math import sqrt

class MapModelTest(unittest.TestCase):
    def setUp(self):
        self.map_model = MapModel()

    def testMoveCoordinateFromOrigin(self):
        x, y = (0, 0)
        angle = 45
        distance = 1
        new_x, new_y = self.map_model.move_coordinate(x, y, angle, distance)
        print("new_x = {}, new_y = {}".format(new_x, new_y))
        corr_x, corr_y = (sqrt(2)/2, sqrt(2) / 2)
        self.assertTrue(abs(corr_x - new_x) <= 0.0001)
        self.assertTrue(abs(corr_y - new_y) <= 0.0001)

    def testMoveCoordinateFromPoint(self):
        x, y = (2, 1)
        angle = -30 - 90
        distance = 3
        new_x, new_y = self.map_model.move_coordinate(x, y, angle, distance)
        print("new_x = {}, new_y = {}".format(new_x, new_y))
        corr_x, corr_y = (2 - 3.0 / 2, 1 - 3.0 * sqrt(3) / 2)
        self.assertTrue(abs(corr_x - new_x) <= 0.0001)
        self.assertTrue(abs(corr_y - new_y) <= 0.0001)

    def testGetValidPoint(self):
        point = self.map_model.get_valid_point()
        self.assertTrue(type(point) == tuple)

if __name__ == '__main__':
    unittest.main()
