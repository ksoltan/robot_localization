import rospy
from helper_functions import TFHelper
from geometry_msgs.msg import PoseStamped, PoseArray
import tf
from math import degrees, atan2, radians, degrees, sqrt, cos, sin
from Particle import Particle
'''
Particles should be propagated by using the changes in odom position.
This test script will use the odom readings from simple map bag files
and propagate one pose through time.

'''
class PropagationTest(object):
    def __init__(self):
        rospy.init_node("propagation_test_node")

        self.tf_helper = TFHelper()

        self.base_link_pose = PoseStamped()
        self.base_link_pose.header.frame_id = "base_link"
        self.base_link_pose.header.stamp = rospy.Time(0)

        self.last_odom_pose = PoseStamped()
        self.last_odom_pose.header.frame_id = "odom"
        self.last_odom_pose.header.stamp = rospy.Time(0)

        self.particle_pose_pub = rospy.Publisher('/particle_pose_array', PoseArray, queue_size=10)
        self.odom_pose_pub = rospy.Publisher('/odom_pose', PoseArray, queue_size=10)

        self.pose_array = PoseArray()
        self.pose_array.header.stamp = rospy.Time(0)
        self.pose_array.header.frame_id = "odom"

        self.p = Particle(x=0,y=0,theta=180, weight=0)
        self.p_array = PoseArray()
        self.p_array.header.stamp = rospy.Time(0)
        self.p_array.header.frame_id = "odom"

        self.is_first = True

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_helper.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                # print("Transform: Trans: {} \n Rot: {}".format(trans, rot))

                # Get odom change.
                new_odom_pose = self.tf_helper.tf_listener.transformPose('odom', self.base_link_pose)
                self.pose_array.poses.append(new_odom_pose.pose)
                self.odom_pose_pub.publish(self.pose_array)
                new_odom_x, new_odom_y, new_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(new_odom_pose.pose)
                last_odom_x, last_odom_y, last_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(self.last_odom_pose.pose)

                x_change = new_odom_x - last_odom_x
                y_change = new_odom_y - last_odom_y
                angular_change = self.tf_helper.angle_diff(new_odom_theta, last_odom_theta) # radians

                print("x: {}, y: {}, theta: {}".format(x_change, y_change, degrees(angular_change)))
                if not self.is_first:
                    self.propagate(x_change, y_change, angular_change)

                self.last_odom_pose = new_odom_pose
                self.is_first = False

                # x = raw_input("Press Enter to continue")
                r.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def propagate(self, dx, dy, dtheta):
        # Update the last odom pose in the process
        r = sqrt(dx**2 + dy**2)
        angle = radians(self.p.theta) # Add p.theta to account for particle's rotation
        # print("p.theta: {} + new_angle: {} = angle: {}".format(radians(self.p.theta), atan2(dy, dx), angle))
        self.p.x += r * cos(angle)
        self.p.y += r * sin(angle)
        self.p.theta = (self.p.theta + degrees(dtheta)) % 360 # Wrap angle

        self.p_array.poses.append(self.p.get_pose())
        self.particle_pose_pub.publish(self.p_array)
        # print(self.p)


if __name__ == "__main__":
    pt = PropagationTest()
    pt.p_array.poses.append(pt.p.get_pose())
    pt.particle_pose_pub.publish(pt.p_array)
    pt.run()
