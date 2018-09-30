import numpy as np
from Particle import Particle

class ParticleDistribution(object):
    def __init__(self, num_particles=200):
        self.particle_list = []
        self.num_particles = num_particles

    '''
    Function: init_particles
    Inputs: MapModel map_model

    Generates a random distribution of particles across the known map.
    Particle positions must be valid points on the map.
    All particles have equal weight.

    '''
    def init_particles(self, map_model):
        # Clear list before appending new particles
        self.particle_list = []
        # generate initial list of hypothesis (particles)
        for i in range(self.num_particles):
            # Find a random valid point on the map
            pos = map_model.get_valid_point()
            # Set all particle weights to be equal
            weight = 1.0 / self.num_particles
            # Add new particle to list
            self.particle_list.append(Particle(pos=pos, weight=weight))

    '''
    Function: resample
    Inputs:

    Regenerates particle distribution based on the relative weights of particles.
    The same number of particles are chosen, but more particles will be chosen
    near more likely positions.

    '''
    def resample(self):
        # Generate a new list of particles, selecting based on their relative weights
        # Use the normalized weights for np.choice, it expects probabilities to sum to 1
        self.particle_list = np.random.choice(self.particle_list, self.num_particles, p=self.get_normalized_weights())

    '''
    Function: get_weights
    Inputs:

    Returns a list of particles' weights, in the order in which they appear in
    the distribution.

    '''
    def get_weights(self):
        particle_weights = []
        for p in self.particle_list:
            particle_weights.append(p.weight)
        return particle_weights

    '''
    Function: get_normalized_weights
    Inputs:

    Returns a list of weights which sum to 1, but have the same proportionality to each other.

    '''
    def get_normalized_weights(self):
        weights = self.get_weights()
        total = sum(weights) * 1.0
        return [w / total for w in weights]


    '''
    Function: print_distribution
    Inputs:

    Display the particle list.

    '''
    def print_distribution(self):
        for p in self.particle_list:
            print p
