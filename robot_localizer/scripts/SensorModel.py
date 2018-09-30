
class SensorModel(object):
    def __init__(self):

    '''
    Function: update_particle_weights
    Inputs: LaserScan scan_ranges
            ParticleDistribution p_distrib
            MapModel map_model

    Assign weights to particles based on how likely the laser scan readings would
    have come from their respective positions.

    '''
    def update_particle_weights(self, scan_ranges, p_distrib, map_model):
        for p in p_distrib:
            p.weight = self.get_how_likely(scan_ranges, p.pos, map_model)

    '''
    Function: get_how_likely
    Inputs: LaserScan scan_ranges
            Tuple pos: (x, y, theta) (in robot_base)
            MapModel map_model

    Return how likely it is to observe the laser scan from the given position.
    Simulate the laser scanner by sweeping through 0 to 360 degree angles from
    the orientation of the position (theta) and finding the nearest obstacle from
    the map.
    Find the difference between the simulated and real readings and assign a likelihood
    inversely proportional to how large this difference is. The smaller the distance,
    the higher the likelihood.
    '''
    def get_how_likely(self, scan_ranges, pos, map_model):
        angles = range(len(scan_ranges))
