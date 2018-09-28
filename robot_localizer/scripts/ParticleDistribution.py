#/usr/bin/env python

# dependencies up here

import numpy as np

class ParticleDistribution(object):
    def __init__(self):
        self.particle_list = []
        self.num_particles = 200 #or something
        self.num_hypotheses = 200 # get them from somewhere?

    def init_particles(self, map_model):
        # generate list
        for x in range(self.num_particles):
            pos = map_model.get_valid_point()
            weight = 1/self.num_particles
            particle = Particle(pos=pos, weight=weight)
            self.particle_list.append(particle)

    def resample(self):
        # todo: shuffle things around
        # create list of indicies
        particle_pos_indices = np.random.choice(self.num_hypotheses,self.num_particles, self.get_weights())
        new_particles = []
        # assign particles to indices
        for p in self.num_particles:
            index = particle_pos_indices[p]
            particle = self.particle_list[index]
            new_particles.append(particle)
        self.particle_list = new_particles

    def get_weights(self):
        # todo
        particle_weights = []
        for p in self.particle_list:
            weight = p.get_weight()
            particle_weights.append(weight)
        return particle_weights

"""
        Particle Distribution
list of Particle elements
Represents the hypothesis of where our robot is
resample()
Repopulate the list of particles by selecting particles based on the new weight distribution
