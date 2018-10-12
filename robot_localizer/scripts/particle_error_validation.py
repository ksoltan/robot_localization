#!/usr/bin/env python

"""This visualizes the projected reading of a particle and displays a radius of the 'closed obstacle distance' around it.

roslaunch error_test.launch to run """

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Vector3, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Header, ColorRGBA
from ParticleDistribution import ParticleDistribution
from Particle import Particle
from MotionModel import MotionModel
from SensorModel import SensorModel
from MapModel import MapModel
from helper_functions import TFHelper
from math import sqrt, degrees
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from math import cos,sin,radians
from occupancy_field import OccupancyField

class ParticleErrorValidation(object):
    def __init__(self):
        rospy.init_node('error_validation_node')
        # Initialize publishers for visualization
        self.error_markers_pub = rospy.Publisher('/error_markers', MarkerArray, queue_size=10)
        
        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)


        # Class initializations
        self.map_model = MapModel()
        self.tf_helper = TFHelper()

        """for error validation"""
        self.particle = Particle(x=0,y=0,theta=0,weight=1)
        self.sample_ranges = np.ones(361)
        self.predicted_obstacle_x = 0.0
        self.predicted_obstacle_y = 0.0

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.tf_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.tf_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def get_predicted_obstacle_error(self, distance_reading, x, y, angle):
        # Predict location of objects based on laser scan reading.
        self.predicted_obstacle_x, self.predicted_obstacle_y = self.move_coordinate(
                                                    x, y, angle, distance_reading)

        # Find the closest obstacle to the predicted obstacle position
        predicted_reading = self.map_model.occupancy_field.get_closest_obstacle_distance(
                                    self.predicted_obstacle_x, self.predicted_obstacle_y)
        #print("Predicted x: {}, y: {}, reading: {}".format(predicted_obstacle_x, predicted_obstacle_y, predicted_reading))
        return 2*predicted_reading

    def move_coordinate(self, x, y, angle, distance):
        return (x + cos(radians(angle)) * distance, y + sin(radians(angle)) * distance)

    def publish_marker_array(self):
        # universal constraints
        header = Header(frame_id = "map",stamp=rospy.Time.now())
        type = 2
        # publish
        # particle marker: RED
        particle_x = self.particle.x
        particle_y = self.particle.y
        particle_point = Point(particle_x,particle_y,0.0)
        particle_scale = Vector3(.1,.1,.1)
        particle_color = ColorRGBA(255,0,0,1)
        # error constants
        error_color = ColorRGBA(0,0,255,.1)
        # predicted marker constants (same as error marker but smaller and diff color): GREEN
        predicted_scale = Vector3(.1,.1,.1)
        predicted_color = ColorRGBA(0,255,0,1)
        
        # create markers
        x = 0
        marker_array = MarkerArray()
        while x<360:
            self.particle.theta = x
            # error marker: BLUE
            error_dist = self.get_predicted_obstacle_error(self.sample_ranges[x],particle_x,particle_y,self.particle.theta)
            error_x = self.predicted_obstacle_x
            error_y = self.predicted_obstacle_y
            error_point = Point(error_x,error_y,0.0)
            error_scale = Vector3(error_dist,error_dist,error_dist)
            # create markers
            particle_id = x
            particle_marker = Marker(header=header,pose=Pose(position=particle_point),color=particle_color,type=type, scale=particle_scale,id=particle_id)
            error_id = x+361
            error_marker = Marker(header=header,pose=Pose(position=error_point),color=error_color,type=type, scale=error_scale,id=x+error_id)
            predicted_id = x+722
            predicted_marker = Marker(header=header,pose=Pose(position=error_point),color=predicted_color,type=type, scale=predicted_scale,id=predicted_id)
            marker_array.markers.append(predicted_marker)
            marker_array.markers.append(particle_marker)
            marker_array.markers.append(error_marker)
            x+=90

        #print(len(marker_array))
        # publish
        self.error_markers_pub.publish(marker_array)

    def run(self):
        # Send the first map to odom transform using the 0, 0, 0 pose.
        self.tf_helper.fix_map_to_odom_transform(Pose(), rospy.Time(0))
        while not rospy.is_shutdown():
            # continuously broadcast the latest map to odom transform
            # Changes to the map to base_link come from our pose estimate from
            # the particle filter.

            self.tf_helper.send_last_map_to_odom_transform()
            self.publish_marker_array()

if __name__ == "__main__":
    p_error = ParticleErrorValidation()
    p_error.run()
