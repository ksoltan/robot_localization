import rospy

"""
Functions
predict - updates positions in p_distrib
move - (todo: noise) - moves single pos based on current velocities
get_delta_t - checks time change from last timestep

"""
class MotionModel(object):
    def __init__(self):
        self.last_time_updated = 0.0 # keeping track of last time the velocity changed

    """
    Function: predict
    Inputs: cmd_vel, p_distrib
    Calls:
    Returns: p_distrib

    Updates positions for all particles in p_distrib
    """

    def predict(cmd_vel, p_distrib):
        for p in p_distrib:
            new_pos = move(p.pos, cmd_vel)
            p.pos = new_pos

    """
    Function: move
    Inputs: pos, cmd_vel
    Calls: get_delta_t

    Updates position based on time since last velocity update
    """

    def move(pos, cmd_vel):
        # todo: add in accuracy model
        delta_t = self.get_delta_t()
        pos.x += cmd_vel.x * cos(pos.theta) * delta_t
        pos.y += cmd_vel.x * sin(pos.theta) * delta_t
        pos.theta += pos.theta * delta_t # maybe something about angular speed??

    """
    Function: get_delta_t
    Inputs: cmd_vel_t

    Records time step and returns delta from last one
    """

    def get_delta_t():
        # get current time in float secs.necs
        current_time = rospy.Time.now()
        current_time = current_time.secs+(current_time.nsecs*1e-09)
        # subtract last time updated from current time
        delta_t = current_time - self.last_time_updated
        self.last_time_updated = current_time
        return delta_t
