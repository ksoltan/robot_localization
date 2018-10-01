#/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import LaserScan


class ParticleFilter(object):
    def __init__(self):
        rospy.init_node('Particle Filter Node')

        # Initialize subscribers to sensors and motors
        #TODO: Check cmd_vel topic. cmd_vel is not stamped.
        rospy.Subscriber('/cmd_vel', TwistStamped, self.get_cmd_vel)
        rospy.Subscriber('/scan', LaserScan, self.read_sensor)

        # Initilize attributes to save latest messages
        self.stamped_cmd_vel = None
        self.scan_ranges = None

        # Particle filter attributes
        self.p_distrib = ParticleDistribution()
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.map_model = MapModel()

    def get_cmd_vel(self, cmd_vel_msg):
        self.stamped_cmd_vel = cmd_vel_msg

    def read_sensor(self, scan_msg):
        self.scan_ranges = scan_msg.ranges

    def run_filter(self):
        # Update particle weights based on the sensor readings.
        self.sensor_model.update_particle_weights(
                            self.scan_ranges, self.p_distrib, self.map_model)

        # Resample the particle distribution
        self.p_distrib.resample()

        # Propagate each particle with the motion model
        self.motion_model.predict(self.stamped_cmd_vel, self.p_distrib)
        """
        run()
        Update weights with sensor model class (takes readings, particle distribution, map)
        Resample particle distribution with particle distribution class
        Visualize current guess
        Predict next step with motion model (takes cmd_vel, particle distribution)
        Visualize next step
        Current step = next step
        """
