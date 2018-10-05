#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from ParticleDistribution import ParticleDistribution
from MotionModel import MotionModel
from SensorModel import SensorModel
from MapModel import MapModel

class ParticleFilter(object):
    def __init__(self):
        rospy.init_node('pf_node')

        # Initialize subscribers to sensors and motors
        rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)
        rospy.Subscriber('/scan', LaserScan, self.read_sensor)
        # Initialize publishers for visualization
        self.particle_pose_pub = rospy.Publisher('/particle_poses', MarkerArray, queue_size=10)
        self.particle_obstacles_pub = rospy.Publisher('/particle_obstacles', Marker, queue_size=10)

        # Initilize attributes to save latest messages
        self.cmd_vel = None
        self.scan_ranges = None
        self.latest_cmd_vel = None
        self.latest_scan_ranges = None
        # Class initializations
        self.p_distrib = ParticleDistribution()
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.map_model = MapModel()

        # After map model has been initialized, create the initial particle distribution
        self.p_distrib.init_particles(self.map_model)
        self.particle_pose_pub.publish(self.p_distrib.get_particle_marker_array())

    def get_cmd_vel(self, cmd_vel_msg):
        self.latest_cmd_vel = cmd_vel_msg

    def read_sensor(self, scan_msg):
        self.latest_scan_ranges = scan_msg.ranges

    def run_filter(self):
        # Update particle weights based on the sensor readings.
        if(self.scan_ranges != None):
            # print("Scan ranges: 0: {}, 90: {}, 180: {}, 270: {}".format(self.scan_ranges[0], self.scan_ranges[90], self.scan_ranges[180], self.scan_ranges[270]))
            self.p_distrib.print_distribution()
            self.sensor_model.update_particle_weights(
                            self.scan_ranges, self.p_distrib.particle_list, self.map_model)
            self.p_distrib.normalize_weights()
            print("Updated with weights")
            self.p_distrib.print_distribution()
            # Display the new distribution
            self.particle_pose_pub.publish(self.p_distrib.get_particle_marker_array())
            # Resample the particle distribution
            print("Resample")
            self.p_distrib.resample()
            self.p_distrib.normalize_weights()

        self.particle_pose_pub.publish(self.p_distrib.get_particle_marker_array())

        # if(self.cmd_vel != None):
        #     print("Cmd_vel: x: {}, y: {}, theta: {}".format(self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z))
        #     # Propagate each particle with the motion model
        #     print("Old distribution")
        #     self.p_distrib.print_distribution()
        #     print("Propagating the particle.")
        #     self.motion_model.predict(self.cmd_vel, self.p_distrib.particle_list)
        #     self.p_distrib.print_distribution()

    def run(self):
        last_time_updated = rospy.get_time()
        while not rospy.is_shutdown():
            if(rospy.get_time() - last_time_updated > 1):
                if(self.latest_cmd_vel != None):
                    self.cmd_vel = self.latest_cmd_vel
                if(self.latest_scan_ranges != None):
                    self.scan_ranges = self.scan_ranges
                self.run_filter()

                last_time_updated = rospy.get_time()
                self.cmd_vel = self.latest_cmd_vel
                self.scan_ranges = self.latest_scan_ranges

if __name__ == "__main__":
    pf = ParticleFilter()
    pf.run()
