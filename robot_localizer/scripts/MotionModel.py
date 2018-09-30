# Things I don't understand:
"""
Where all the variables are coming from
does this need an init?
where are the selfs?
How do @s work?
-- we should have a big documentation file of classes
"""

"""
Function: prev_time
Inputs: none?
Calls: none
Returns: prev_time

Represents the last time we updated the motion
"""

class MotionModel(object):
    def __init__(self, cmd_vel, p_distrib):


    def prev_time(self):

    """
    Function: predict
    Inputs: cmd_vel, p_distrib
    Calls:
    Returns: p_distrib

    updates positions for all particles in p_distrib
    --- pos inputs? no noise for now
    """

    def predict(cmd_vel, p_distrib):
        for each p in p_distrib:
            new_pos = move(p.pos, cmd_vel)
            p.pos = new_pos
        return p_distrib

    """
    Function: move
    Inputs: pos, cmd_vel
    Calls: get_delta_t

    Updates position based on time since last velocity update
    """

    def move(pos, cmd_vel):
        # todo: add in accuracy model
        delta_t = get_delta_t(cmd_vel.t)
        pos.x += cmd_vel.x *cos(pos.theta) * delta_t
        pos.y += cmd_vel.x * sin(pos.theta) * delta_t
        pos.theta += pos.theta * delta_t # maybe something about angular speed??

    """
    Function: get_delta_t
    Inputs: cmd_vel_t
    Calls: prev_time
    """

    def get_delta_t(cmd_vel_t):
        if(cmd_vel_t - prev_time) == 0:
            delta_t = Time.now() - prev_time
            prev_time = Time.now()
            return delta_t
        return cmd_vel_t - prev_time