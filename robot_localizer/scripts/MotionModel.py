import rospy
from math import cos, sin, sqrt, radians, degrees, atan2
from geometry_msgs.msg import PoseStamped, PoseArray
import tf

"""
Functions
predict - updates positions in particle_list
move - (todo: noise) - moves single pos based on current velocities
get_delta_t - checks time change from last timestep

"""
class MotionModel(object):
    def __init__(self):
        self.last_odom_pose = PoseStamped()
        self.last_odom_pose.header.frame_id = 'odom'
        self.last_odom_pose.header.stamp = rospy.Time(0)
        # Point to perform odom transform on.
        self.base_link_pose = PoseStamped()
        self.base_link_pose.header.stamp = rospy.Time(0)
        self.base_link_pose.header.frame_id = 'base_link'

        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "map"
        self.pose_array.header.stamp = rospy.Time(0)

    '''
    Function: get_change_in_motion
    Inputs: TFHelper tf_helper
            bool update_last_pose: (default false)

    Return a tuple (x_change, y_change, angular_change) calculating the change between
    the previous robot's pose in odom to the newest one.
    The x_change and y_change are the change in the x and y positions, respectively.
    The angular change is the change in orientation in radians.

    Set the update_last_pose flag to True to save the last pose (to cut down on transformPose lookups).

    '''
    def get_change_in_motion(self, tf_helper, update_last_pose=False):
        # Get the latest odom pose
        try:
            # (trans,rot) = tf_helper.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            # print("Got the transform")

            new_odom_pose = tf_helper.tf_listener.transformPose('odom', self.base_link_pose)
            print("New Odom Pose: {}".format(new_odom_pose))
            new_odom_x, new_odom_y, new_odom_theta = tf_helper.convert_pose_to_xy_and_theta(new_odom_pose.pose)
            last_odom_x, last_odom_y, last_odom_theta = tf_helper.convert_pose_to_xy_and_theta(self.last_odom_pose.pose)

            print("new_odom_x: {}, new_odom_y: {}, new_odom_theta: {}".format(new_odom_x, new_odom_y, new_odom_theta))

            x_change = new_odom_x - last_odom_x
            y_change = new_odom_y - last_odom_y
            angular_change = tf_helper.angle_diff(new_odom_theta, last_odom_theta) # radians

            if(update_last_pose):
                self.last_odom_pose = new_odom_pose

            return (x_change, y_change, angular_change)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return (0, 0, 0)

    '''
    Function: has_moved_enough
    Inputs: TFHelper tf_helper
            float distance_moved_thresshold
            float angle_turned_threshold (degrees)

    Return true if the robot has either moved more than the linear threshold
    distance-wise (between odom poses) or more than the angular threshold rotation-wise.

    '''
    def has_moved_enough(self, tf_helper, distance_moved_threshold, angle_turned_threshold):
        x_change, y_change, angular_change = self.get_change_in_motion(tf_helper)

        # Calculate Cartesian distance moved
        linear_change = sqrt(x_change**2 + y_change**2)

        return linear_change > distance_moved_threshold or degrees(angular_change) > angle_turned_threshold

    """
    Function: predict
    Inputs: cmd_vel, particle_list
    Calls:
    Returns: particle_list

    Updates positions for all particles in particle_list base on the change in odom pose.
    Updates the last saved odom pose in motion model.
    """
    def propagate(self, particle_list, tf_helper):
        # Update the last odom pose in the process
        dx, dy, dtheta = self.get_change_in_motion(tf_helper, update_last_pose=True)
        print("dx: {}, dy: {}, dtheta: {}".format(dx, dy, dtheta))
        for p in particle_list:
            # Get the pose change by propagating in the particle's frame of reference
            # Change odom change to polar coordinates, rotating into particle's frame
            r = sqrt(dx**2 + dy**2)
            angle = radians(p.theta)
            p.x += r * cos(angle)
            p.y += r * sin(angle)
            p.theta = (p.theta + degrees(dtheta)) % 360 # Wrap angle
            # print(p)

    def get_pose_array(self):
        # Add the latest pose
        self.pose_array.poses.append(self.last_odom_pose.pose)
        return self.pose_array
