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

        # To test random selection, would run this resampling multiple times
        # and look at statistics of each selection and compare to relative weightsself.
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

    def testNormalizeWeights(self):
        for p in self.p_distrib.particle_list:
            p.weight = random.uniform(0, 1)

        self.p_distrib.normalize_weights()

        correct_weights = self.p_distrib.get_normalized_weights()
        weights = self.p_distrib.get_weights()
        [self.assertTrue((abs(weights[i] - correct_weights[i])) <= 0.0001) for i in range(len(weights))]

    def testNormalizedWeightsAfterResampling(self):
        for p in self.p_distrib.particle_list:
            p.weight = random.uniform(0, 0.5)
        self.p_distrib.normalize_weights()
        print("Distrib before resampling")
        self.p_distrib.print_distribution()
        print("Weight sum before resampling: {}".format(sum(self.p_distrib.get_weights())))

        self.p_distrib.resample()
        print("Distrib after resampling")
        self.p_distrib.print_distribution()
        print("Weight sum after resampling: {}".format(sum(self.p_distrib.get_weights())))

        self.p_distrib.normalize_weights()
        print("Distrib after normalizing")
        self.p_distrib.print_distribution()
        print("Weight sum after normalizing: {}".format(sum(self.p_distrib.get_weights())))
        self.assertTrue(abs(1 - sum(self.p_distrib.get_weights())) <= 0.0001)

    def testSpecificParticleNormalizedWeightsAfterResampling(self):
        self.p_distrib.particle_list = [Particle(x=1.1026683979627143, y=1.3169624148597046, theta=204, weight=0.0409339465492),
                                        Particle(x=0.4660179516836995, y=0.3570517416074323, theta=305, weight=0.199876196252),
                                        Particle(x=1.9121964510843559, y=0.49645320731338294, theta=209, weight=0.1986675867),
                                        Particle(x=1.1026683979627143, y=1.3169624148597046, theta=204, weight=0.0409339465492),
                                        Particle(x=0.20860576164448363, y=0.2618577992238944, theta=318, weight=0.199474254139)]
        print("Weird Distrib")
        self.p_distrib.print_distribution()

        self.p_distrib.normalize_weights()
        self.assertTrue(abs(1 - sum(self.p_distrib.get_weights())) <= 0.0001)




if __name__ == '__main__':
    unittest.main()
