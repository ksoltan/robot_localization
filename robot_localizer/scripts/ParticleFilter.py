#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
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
        rospy.Subscriber('/scan', LaserScan, self.read_sensor)
        # Initialize publishers for visualization
        self.particle_pose_pub = rospy.Publisher('/particle_pose_array', PoseArray, queue_size=10)
        self.odom_pose_pub = rospy.Publisher('odom_pose', PoseArray, queue_size=10)
        # self.particle_obstacles_pub = rospy.Publisher('/particle_obstacles', Marker, queue_size=10)

        self.latest_scan_ranges = []

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
        self.particle_pose_pub.publish(self.p_distrib.get_particle_pose_array())

    '''
    Function: read_sensor
    Inputs: LaserScan scan_msg

    Save the ranges of the laser scanner.

    '''
    def read_sensor(self, scan_msg):
        self.latest_scan_ranges = scan_msg.ranges

    '''
    Function: update_pose_estimate
    Inputs:

    Returns a new pose estimate after running a particle filter. Called if the robot has
    moved or turned enough to merit a new pose estimate.
    The current particles (representing hypothetical poses) are propagated based on the
    change in odom pose of the robot (how much it has moved since the last estimate). Each
    new particle is assigned a new weight based on how likely it is to observe the laser
    scan values.
    The pose estimate is a weighted average of all of the particle's poses.

    '''
    def update_pose_estimate(self):
        # Propagate the particles because the robot has moved. The sensor update
        # should happen on the new poses.
        # Display the new pose
        self.odom_pose_pub.publish(self.motion_model.get_pose_array())
        self.motion_model.propagate(self.p_distrib.particle_list, self.tf_helper)
        # self.p_distrib.print_distribution()
        pose_array = self.p_distrib.get_particle_pose_array()
        pose_array.header.stamp = rospy.Time(0)
        self.particle_pose_pub.publish(pose_array)

        # Update particle weights based on the sensor readings.
        if(self.latest_scan_ranges != []):
            scan_ranges = self.latest_scan_ranges # Assuming this will not change the object if we get a new scan.
            self.sensor_model.update_particle_weights(scan_ranges, self.p_distrib.particle_list, self.map_model)
            self.p_distrib.normalize_weights()
            # Resample the particle distribution
            print("Resample")
            self.p_distrib.resample()
            self.p_distrib.normalize_weights()
            # Display the new distribution
            pose_array = self.p_distrib.get_particle_pose_array()
            pose_array.header.stamp = rospy.Time(0)
            self.particle_pose_pub.publish(pose_array)


        new_pose_estimate = self.p_distrib.get_pose_estimate() # Just Pose, not stamped
        return new_pose_estimate

    def publish_map_markers(self):
        # figure out map origin
        return

    def run(self):
        while not rospy.is_shutdown():
            # continuously broadcast the latest map to odom transform
            # Changes to the map to base_link come from our pose estimate from
            # the particle filter.
            self.tf_helper.send_last_map_to_odom_transform()
            self.tf_helper.fix_map_to_odom_transform(Pose(), rospy.Time(0))

            if(self.motion_model.has_moved_enough(self.tf_helper, self.distance_moved_threshold, self.angle_turned_threshold)):
                # Run the particle filter
                new_pose_estimate = self.update_pose_estimate()
                # Update the map to odom transform using new pose estimate
                self.tf_helper.fix_map_to_odom_transform(new_pose_estimate, rospy.Time(0))

if __name__ == "__main__":
    pf = ParticleFilter()
    pf.run()
