#/usr/bin/env python

# dependencies up here

class ParticleFilter(object):
    def __init__(self):
        rospy.init_node('Particle Filter Node')
        # Subscribers
        rospy.Subscriber('/cmd_vel', TwistStamped, self.get_cmd_vel)
        rospy.Subscriber('/scan', LaserScan, self.read_sensor)
        # save subscriptions
        self.stamped_cmd_vel = None
        self.scan_ranges = None
        # Class initializations
        self.p_distrib = ParticleDistribution()
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.map_model = MapModel()

    def get_cmd_vel(self, cmd_vel_msg):
        self.stamped_cmd_vel = cmd_vel_msg
    
    def read_sensor(self, laser_msg):
        self.scan_ranges = laser_msg.ranges

    def run(self):
        """
        run()
        Update weights with sensor model class (takes readings, particle distribution, map)
        Resample particle distribution with particle distribution class
        Visualize current guess
        Predict next step with motion model (takes cmd_vel, particle distribution)
        Visualize next step
        Current step = next step
        """