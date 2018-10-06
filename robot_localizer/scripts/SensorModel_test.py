from math import sqrt
from occupancy_field import OccupancyField

# Assumptions: MapModel is a good map
# testing function: self.sensor_model.update_particle_weights(
#        scan_ranges, 
#        self.p_distrib.particle_list, 
#        self.map_model)
# 


class SensorModelTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("SensorModelTest")

        self.map_model = MapModel()
        self.sensor_model = SensorModel()
        self.particle_list = 

    # the idea: check the likelihood of each particle's readings and assign weight based on them
    def PerfectParticleReadings(self):




"""what do I want to test?
- whether particle weights are updating accurately

test cases:
- in map corner
- in map middle
