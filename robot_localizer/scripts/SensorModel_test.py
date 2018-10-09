import unittest
import rospy
from math import sqrt
from occupancy_field import OccupancyField
from Particle import Particle
from MapModel import MapModel
from SensorModel import SensorModel

# Assumptions: MapModel is a good map
# testing function: self.sensor_model.update_particle_weights(
#        scan_ranges, 
#        self.p_distrib.particle_list, 
#        self.map_model)
# 
# Comes before normalization step:
# 

"""
Questions: why is map_model.get_valid_points() returning a random between 0 and 2?
"""

"""
Needs pf_test.launch running for the map_model to be valid
"""

class Sketches(object):
    def __init__(self):
        pass

    def sketch_get_weights(self, particle_list):
        particle_weights = []
        for p in particle_list:
            particle_weights.append(p.weight)
        return particle_weights


    def create_all_same_particles(self):
        particle_list = []
        for i in range(100): # create 100 particles
            x, y = 0.0, 0.0
            theta = 0.0
            weight = 1.0 / 100
            particle_list.append(Particle(x=x,y=y,theta=theta,weight=weight))
        return particle_list


class SensorModelTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("SensorModelTest")

        self.helper = Sketches()

        self.map_model = MapModel()
        self.sensor_model = SensorModel()
        self.particle_list = []
        self.scan_ranges = [1 for x in range(361)]

    # the idea: check the likelihood of each particle's readings and assign weight based on them
    
    def test_uniform_particle_weights(self): # all updated weights should be the same
        # all particles the same
        self.particle_list = self.helper.create_all_same_particles()
        # get_how_likely only uses some of the angles for now
        self.sensor_model.update_particle_weights(
                self.scan_ranges, self.particle_list, self.map_model)
        # turn array of weights into set (only carries one instance of a thing) and if it's only one long, there was only the same thing in the array
        weight_set = set(self.helper.sketch_get_weights(self.particle_list))
        self.assertTrue(len(weight_set) == 1)


if __name__ == '__main__':
    unittest.main()

"""what do I want to test?
- whether particle weights are updating accurately

test cases:
- in map corner
- in map middle
"""