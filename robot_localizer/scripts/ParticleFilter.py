#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from ParticleDistribution import ParticleDistribution
from MotionModel import MotionModel
from SensorModel import SensorModel
from MapModel import MapModel
from helper_functions import TFHelper
from math import sqrt, degrees

class ParticleFilter(object):
    def __init__(self):
        rospy.init_node('pf_node')

        # Initialize subscribers to sensors and motors
        rospy.Subscriber('/odom', Odometry, self.get_odom_pose)
        rospy.Subscriber('/scan', LaserScan, self.read_sensor)
        # Initialize publishers for visualization
        self.particle_pose_pub = rospy.Publisher('/particle_poses', MarkerArray, queue_size=10)
        self.particle_obstacles_pub = rospy.Publisher('/particle_obstacles', Marker, queue_size=10)

        # Initilize attributes to save latest messages
        self.scan_ranges = []
        self.latest_scan_ranges = []

        self.latest_odom_x = None
        self.latest_odom_y = None
        self.latest_odom_theta = None

        self.current_odom_x = 0
        self.current_odom_y = 0
        self.current_odom_theta = 0

        # Class initializations
        self.p_distrib = ParticleDistribution()
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.map_model = MapModel()
        self.tf_helper = TFHelper()

        # When to run the particle filter
        self.distance_moved_threshold = 0.2 # m
        self.angle_turned_threshold = 10 # deg

        # After map model has been initialized, create the initial particle distribution
        self.p_distrib.init_particles(self.map_model)
        self.particle_pose_pub.publish(self.p_distrib.get_particle_marker_array())

    def get_odom_pose(self, odom_msg):
        self.latest_odom_x, self.latest_odom_y, self.latest_odom_theta =
                        self.tf_helper.convert_pose_to_xy_and_theta(odom_msg.pose.pose)
        # Convert angle to degrees
        self.latest_odom_theta = degrees(self.latest_odom_theta)

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

        if(self.current_odom_x != None):
            print("Odom: x: {}, y: {}, theta: {}".format(self.current_odom_x, self.current_odom_y, self.current_odom_theta))
            # Propagate each particle with the motion model
            print("Old distribution")
            self.p_distrib.print_distribution()
            print("Propagating the particle.")
            self.motion_model.predict((self.current_odom_x, self.current_odom_y, self.current_odom_theta), self.p_distrib.particle_list)
            self.p_distrib.print_distribution()

    def has_moved_enough(self):
        # Return true if the robot has moved above a certain threshhold either
        # linearly or angularly.
        if(self.latest_odom_x == None):
            return False
        distance_moved = sqrt((self.latest_odom_x - self.current_odom_x)**2 + (self.latest_odom_y - self.current_odom_y)**2)
        angle_turned = self.latest_odom_theta - self.current_odom_theta

        return (distance_moved > self.distance_moved_threshold) or (angle_turned > self.angle_turned_threshold)

    def run(self):
        while not rospy.is_shutdown():
            # continuously broadcast the latest map to odom transform
            # Changes to the map to base_link come from our pose estimate from
            # the particle filter.
            self.transform_helper.send_last_map_to_odom_transform()

            if(self.has_moved_enough()):
                # Set the new odom
                self.current_odom_x = self.latest_odom_x
                self.current_odom_y = self.latest_odom_y
                self.current_odom_theta = self.latest_odom_theta

                self.run_filter()

                self.scan_ranges = self.latest_scan_ranges

if __name__ == "__main__":
    pf = ParticleFilter()
    pf.run()
