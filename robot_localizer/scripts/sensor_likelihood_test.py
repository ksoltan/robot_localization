'''
To run: Run pf_test.launch with the use_builtin set to true. Make sure to start the bag file before running the next line.
        Run python sensor_likelihood_test.py
'''
import rospy
from math import sin, cos, radians, degrees, atan2
from helper_functions import TFHelper
from Particle import Particle
from occupancy_field import OccupancyField
from visualization_msgs.msg import Marker, MarkerArray
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Point, PoseStamped, Pose, PoseWithCovarianceStamped

class SensorLikelihoodTest(object):
    def __init__(self):
        rospy.init_node("sensor_likelihood_test")
        self.occupancy_field = OccupancyField()
        self.tf_helper = TFHelper()

        self.latest_scan_ranges = []
        rospy.Subscriber('/scan', LaserScan, self.read_sensor)

        self.odom_poses = PoseArray()
        self.odom_poses.header.frame_id = "odom"
        self.particle_pose_pub = rospy.Publisher('/particle_pose_array', PoseArray, queue_size=10)
        self.odom_pose_pub = rospy.Publisher('odom_pose', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        self.p = Particle(x=0, y=0, theta=0, weight=1)

        self.particle_poses = PoseArray()
        self.particle_poses.header.frame_id = "map"

        self.base_link_pose = PoseStamped()
        self.base_link_pose.header.frame_id = "base_link"
        self.base_link_pose.header.stamp = rospy.Time(0)
        self.counter = 0

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.tf_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        self.tf_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        self.tf_helper.send_last_map_to_odom_transform()
        # initialize your particle filter based on the xy_theta tuple

    def read_sensor(self, scan_msg):
        self.latest_scan_ranges = scan_msg.ranges

    def run(self):
        self.tf_helper.fix_map_to_odom_transform(Pose(), rospy.Time(0))
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                print("Trying")
                (trans,rot) = self.tf_helper.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                # print("Transform: Trans: {} \n Rot: {}".format(trans, rot))
                if(self.latest_scan_ranges != []):
                    new_odom_pose = self.tf_helper.tf_listener.transformPose('odom', self.base_link_pose)
                    self.odom_poses.poses.append(new_odom_pose.pose)
                    self.odom_poses.header.stamp = rospy.Time.now()
                    self.odom_pose_pub.publish(self.odom_poses)

                    new_odom_pose_map = self.tf_helper.tf_listener.transformPose('map', new_odom_pose)

                    # Update particle  to be in odom for check
                    x, y, theta = self.tf_helper.convert_pose_to_xy_and_theta(new_odom_pose_map.pose)

                    self.p.x = x
                    self.p.y = y
                    self.p.theta = theta

                    self.get_how_likely()


                    self.particle_poses.poses.append(self.p.get_pose())
                    self.particle_poses.header.stamp = rospy.Time.now()
                    self.particle_pose_pub.publish(self.particle_poses)

                    # self.tf_helper.send_last_map_to_odom_transform()

                # x = raw_input("Press Enter to continue")
                r.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def get_how_likely(self):
        # angles = range(len(scan_ranges))
        angles = [0, 45] # Use only some of the angles for now
        num_angles = 0
        marker_arr = MarkerArray()

        for angle in angles:
            reading = self.latest_scan_ranges[angle]

            if(reading > 0.0):
                print("Have a valid reading: {} at angle: {}".format(reading, angle))
                num_angles += 1

                # Take into account robot's yaw
                yaw = self.p.theta
                angle_in_map = yaw + angle
                print("x, y, theta: {}, {}, {}".format(self.p.x, self.p.y, angle_in_map))

                predicted_obstacle_x, predicted_obstacle_y = self.move_coordinate(
                                                            self.p.x, self.p.y, angle_in_map, reading)
                # print("Predicted x: {}, y: {}".format(predicted_obstacle_x, predicted_obstacle_y))
                my_marker = Marker()
                my_marker.header.stamp = rospy.Time.now()
                my_marker.header.frame_id = "map"
                my_marker.color.a = 0.5
                my_marker.type = Marker.SPHERE
                my_marker.id = self.counter
                self.counter += 1
                my_marker.pose.position.x = predicted_obstacle_x
                my_marker.pose.position.y = predicted_obstacle_y
                my_marker.lifetime = rospy.Time(1)
                marker_arr.markers.append(my_marker)

                # error = self.get_predicted_obstacle_error(
                #                             reading, pos[0], pos[1], angle_in_map)
                predicted_reading = self.occupancy_field.get_closest_obstacle_distance(
                                            predicted_obstacle_x, predicted_obstacle_y)
                print("Predicted x: {}, y: {}, reading: {}".format(predicted_obstacle_x, predicted_obstacle_y, predicted_reading))
                error = predicted_reading
                if(predicted_reading != predicted_reading): # Check for nan
                    print("Got Nan")
                    my_marker.color.g = 1.0
                    my_marker.scale.x = 0.1
                    my_marker.scale.y = 0.1
                    my_marker.scale.z = 0.1
                else:
                    my_marker.color.b = 1.0 - error
                    my_marker.color.r = error + 0.1
                    my_marker.scale.x = error + 0.1
                    my_marker.scale.y = error + 0.1
                    my_marker.scale.z = error + 0.1


            self.marker_pub.publish(marker_arr)


    def get_predicted_obstacle_error(self, distance_reading, x, y, angle):
        # Predict location of objects based on laser scan reading.
        predicted_obstacle_x, predicted_obstacle_y = self.move_coordinate(
                                                    x, y, angle, distance_reading)

        # Find the closest obstacle to the predicted obstacle position
        predicted_reading = self.occupancy_field.get_closest_obstacle_distance(
                                    predicted_obstacle_x, predicted_obstacle_y)
        print("Predicted x: {}, y: {}, reading: {}".format(predicted_obstacle_x, predicted_obstacle_y, predicted_reading))
        return predicted_reading

    def move_coordinate(self, x, y, angle, distance):
        return (x + cos(radians(angle)) * distance, y + sin(radians(angle)) * distance)


    def get_uniform_probability(self, error):
        # x axis is width
        max_width = 1.0 * self.occupancy_field.map.info.width * map_model.occupancy_field.map.info.resolution / 2.0
        # y axis is height
        max_height = 1.0 * self.occupancy_field.map.info.height * map_model.occupancy_field.map.info.resolution / 2.0
        max_distance = sqrt(max_width**2 + max_height**2)

        return (max_distance - error) / max_distance

if __name__ == "__main__":
    sl = SensorLikelihoodTest()
    sl.run()
