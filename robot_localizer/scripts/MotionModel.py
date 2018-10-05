import rospy
from math import cos, sin, radians
"""
Functions
predict - updates positions in particle_list
move - (todo: noise) - moves single pos based on current velocities
get_delta_t - checks time change from last timestep

"""
class MotionModel(object):
    def __init__(self):
        self.last_time_updated = 0.0 # keeping track of last time the velocity changed

    """
    Function: predict
    Inputs: cmd_vel, particle_list
    Calls:
    Returns: particle_list

    Updates positions for all particles in particle_list
    """

    def predict(self, cmd_vel, particle_list):
        delta_t = self.get_delta_t()
        for p in particle_list:
            new_x, new_y, new_theta = self.move(p, cmd_vel, delta_t)
            p.x = new_x
            p.y = new_y
            p.theta = new_theta

    """
    Function: move
    Inputs: pos, cmd_vel
    Calls: get_delta_t

    Updates position based on time since last velocity update
    """

    def move(self, particle, cmd_vel, delta_t):
        # todo: add in accuracy model
        # delta_t = self.get_delta_t()
        particle.x += cmd_vel.linear.x * cos(radians(particle.theta)) * delta_t
        particle.y += cmd_vel.linear.x * sin(radians(particle.theta)) * delta_t
        particle.theta += particle.theta * delta_t # maybe something about angular speed??
        particle.theta %= 360
        return particle.pos

    """
    Function: get_delta_t
    Inputs: cmd_vel_t

    Records time step and returns delta from last one
    """

    def get_delta_t(self):
        # get current time in float secs.necs
        current_time = rospy.Time.now()
        current_time = current_time.secs+(current_time.nsecs*1e-09)
        # subtract last time updated from current time
        delta_t = current_time - self.last_time_updated
        self.last_time_updated = current_time
        return delta_t
