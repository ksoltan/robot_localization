from math import sqrt

class SensorModel(object):
    def __init__(self):
        pass

    '''
    Function: update_particle_weights
    Inputs: LaserScan scan_ranges
            ParticleDistribution particle_list
            MapModel map_model

    Assign weights to particles based on how likely the laser scan readings would
    have come from their respective positions.

    '''
    def update_particle_weights(self, scan_ranges, particle_list, map_model):
        for p in particle_list:
            p.weight = self.get_how_likely(scan_ranges, p.pos, map_model)
            print(p)

    '''
    Function: get_how_likely
    Inputs: LaserScan scan_ranges
            Tuple pos: (x, y, theta) (in robot_base)
            MapModel map_model

    Return how likely it is to observe the laser scan from the given position.
    Simulate the laser scanner by sweeping through 0 to 360 degree angles from
    the orientation of the position (theta) and finding the nearest obstacle from
    the map.
    Based on the simulated and real readings, assign a likelihood inversely
    proportional to how large this difference is. The smaller the distance,
    the higher the likelihood.
    '''
    def get_how_likely(self, scan_ranges, pos, map_model):
        # angles = range(len(scan_ranges))
        angles = [0, 45, 90, 135, 180, 225, 360] # Use only some of the angles for now
        total_probability = 0
        num_angles = 0
        for angle in angles:
            reading = scan_ranges[angle]
            if(reading > 0.0):
                num_angles += 1
                # Take into account robot's yaw
                yaw = pos[2]
                angle_in_map = yaw + angle
                # Error in obstacle reading for this angle
                error = map_model.get_predicted_obstacle_error(
                                            reading, pos[0], pos[1], angle_in_map)
                if(error != error): # Check for nan
                    print("Got Nan")
                    total_probability += 0
                else:
                    total_probability += self.get_uniform_probability(map_model, error)
        # Add together the probabilities of each angle reading, and average them.
        if(num_angles <= 0):
            return 0
        return total_probability / num_angles

    '''
    Function: get_uniform_probability
    Inputs: MapModel map_model
            float error: (distance between the predicted and actual obstacle)

    Return a probability inversely proportional to the error.
    We assume that the maximum error is the maximum distance between two points
    on the map. This would constitute a probability of 0.
    If the error is 0, there is a definite certainty (probability 1) that the
    given position produced the readings.
    All other probabilities are uniformly distributed between these two extreme
    cases.
    '''
    def get_uniform_probability(self, map_model, error):
        # x axis is width
        max_width = 1.0 * map_model.occupancy_field.map.info.width * map_model.occupancy_field.map.info.resolution
        # y axis is height
        max_height = 1.0 * map_model.occupancy_field.map.info.height * map_model.occupancy_field.map.info.resolution
        max_distance = sqrt(max_width**2 + max_height**2)
        return (max_distance - error) / max_distance
