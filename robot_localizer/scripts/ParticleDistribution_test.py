import unittest
from ParticleDistribution import *
from Particle import Particle
import random

class MapModelSketch(object):
    def get_valid_point(self):
        return (random.randint(1, 10), random.randint(1, 10))

class ParticleFilterTest(unittest.TestCase):
    def setUp(self):
        self.p_distrib = ParticleDistribution(num_particles=10)
        self.p_distrib.init_particles(MapModelSketch())

    def testInit(self):
        self.assertTrue(10, len(self.p_distrib.particle_list))
        [self.assertEqual(p.weight, 1.0 / 10) for p in self.p_distrib.particle_list]

    def testGetWeights(self):
        i = 1
        for p in self.p_distrib.particle_list:
            p.weight = i * 0.1
            i += 1
        correct_weights = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
        weights = self.p_distrib.get_weights()
        [self.assertTrue((abs(weights[i] - correct_weights[i])) <= 0.0001) for i in range(len(weights))]

    def testGetNormalizedWeights(self):
        for p in self.p_distrib.particle_list:
            p.weight = random.uniform(0, 1)
        print("Weights: {}\n Sum: {}".format(self.p_distrib.get_weights(), sum(self.p_distrib.get_weights())))
        normalized_weights = self.p_distrib.get_normalized_weights()
        print("Normalized Weights: {}".format(normalized_weights))
        self.assertTrue(abs(1- sum(normalized_weights)) <= 0.0001)

    def testResample(self):
        # Assign random weights
        for p in self.p_distrib.particle_list:
            p.weight = random.uniform(0, 1)

        print("Distrib")
        self.p_distrib.print_distribution()

        # Resample
        self.p_distrib.resample()

        print("New Distrib")
        self.p_distrib.print_distribution()

        # Not sure what a good way to test random selection is. Using print statements to compare
        self.assertTrue(10, len(self.p_distrib.particle_list))

    def testResampleExtreme(self):
        # Assign all 0 weights except for 1
        for p in self.p_distrib.particle_list:
            p.weight = 0.0
        correct_particle = self.p_distrib.particle_list[0]
        correct_particle.weight = 1.0

        # Resample
        self.p_distrib.resample()
        [self.assertEqual(correct_particle, p) for p in self.p_distrib.particle_list]

if __name__ == '__main__':
    unittest.main()
