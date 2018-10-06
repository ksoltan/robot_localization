import numpy as np
from Particle import Particle
from geometry_msgs.msg import Pose, PoseArray
import random
from math import cos, sin, radians

class ParticleDistribution(object):
    def __init__(self, num_particles=100):
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
            x, y = map_model.get_valid_point()
            theta = random.randint(0, 361)
            # Set all particle weights to be equal
            weight = 1.0 / self.num_particles
            # Add new particle to list
            self.particle_list.append(Particle(x=x, y=y, theta=theta, weight=weight))
        return self.particle_list

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
        particle_pos_indices = np.random.choice(range(self.num_particles),self.num_particles, p=self.get_normalized_weights())
        new_particles = []
        # assign particles to indices
        for i in particle_pos_indices:
            new_particle = self.particle_list[i].make_copy()
            new_particles.append(new_particle)
        self.particle_list = new_particles

    '''
    Function: normalize_weights
    Inputs:

    Normalize the distribution's weights to sum to 1.

    '''
    def normalize_weights(self):
        weight_total = sum(self.get_weights()) * 1.0
        for p in self.particle_list:
            p.weight = p.weight / weight_total
        #TODO: it seems like not all of the particle manipulation happens inside of ParticleDistribution
        # which is inconvenient because normalizing the weights should always happen. Not too bad now, since
        # the only place it is changed is Particle Filter.

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
    Function: get_particle_pose_array
    Inputs:

    Returns a PoseArray representing the particle distribution.

    '''
    def get_particle_pose_array(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"

        for p in self.particle_list:
            pose_array.poses.append(p.get_pose())

        return pose_array

    def get_pose_estimate(self):
        avg_x = 0.0
        avg_y = 0.0
        avg_theta = 0.0

        for p in self.particle_list:
            avg_x += p.weight * p.x
            avg_y += p.weight * p.y
            avg_theta += p.weight * p.theta

        pose = Pose()
        pose.position.x = avg_x / self.num_particles
        pose.position.y = avg_y / self.num_particles
        pose.orientation.w = cos(radians(avg_theta) / self.num_particles / 2.0)
        pose.orientation.z = sin(radians(avg_theta) / self.num_particles / 2.0)

        return pose


    '''
    Function: print_distribution
    Inputs:

    Display the particle list.

    '''
    def print_distribution(self):
        for p in self.particle_list:
            print(p)
