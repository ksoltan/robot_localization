import unittest
import random
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf.transformations as t
from tf import TransformListener
from math import cos, sin, radians, sqrt, atan2
import math

from Particle import Particle
from MotionModel import MotionModel

'''
To run this test:
1. Launch roscore
2. Play some bagfile???
'''
# Define a hacked listener.
class Listener_Sketch(object):
    def __init__(self, dx=0, dy=0, dtheta=0):
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta

    def transformPose(self, to_frame, pose):
        p = PoseStamped()
        p.header.frame_id = to_frame
        p.pose.position.x = pose.pose.position.x + self.dx * cos(self.dtheta) + self.dy * cos(self.dtheta + radians(90))
        p.pose.position.y += pose.pose.position.y + self.dx * sin(self.dtheta) + self.dy * sin(self.dtheta + radians(90))
        p.pose.orientation.w = (pose.pose.orientation.w + cos(self.dtheta / 2.0)) % radians(360)
        p.pose.orientation.z = (pose.pose.orientation.w + sin(self.dtheta / 2.0)) % radians(360)
        return p

    def setNewTransform(self, dx, dy, dtheta):
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta

# Define functions from TFHelper needed in MotionModel
class TFHelper_Sketch(object):
    def __init__(self):
        self.tf_listener = Listener_Sketch()

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = t.euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return atan2(sin(z), cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should
            be in radians) the difference is always based on the closest
            rotation from angle a to angle b.
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

class MotionModelTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("MotionModelTest")

        self.tf_helper = TFHelper_Sketch()
        self.motion_model = MotionModel()

    # def testTFHelperSketchNoMovement(self):
    #     p = PoseStamped()
    #     p.header.frame_id = "base_link"
    #     transformed_p = self.tf_helper.tf_listener.transformPose("odom", p)
    #
    #     self.assertEqual(transformed_p.header.frame_id, "odom")
    #     self.assertEqual(transformed_p.pose.position.x, 0)
    #     self.assertEqual(transformed_p.pose.position.y, 0)
    #     self.assertEqual(transformed_p.pose.orientation.w, 1)
    #     self.assertEqual(transformed_p.pose.orientation.z, 0)
    #
    # def testTFHelperSketchRotate90(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=0, dy=0, dtheta=radians(90))
    #     p = PoseStamped()
    #     p.header.frame_id = "base_link"
    #     transformed_p = self.tf_helper.tf_listener.transformPose("odom", p)
    #
    #     self.assertEqual(transformed_p.header.frame_id, "odom")
    #     self.assertEqual(transformed_p.pose.position.x, 0)
    #     self.assertEqual(transformed_p.pose.position.y, 0)
    #     self.assertTrue(abs(transformed_p.pose.orientation.w - sqrt(2)/2) < 0.0001)
    #     self.assertTrue(abs(transformed_p.pose.orientation.z - sqrt(2)/2) < 0.0001)
    #
    #
    # def testTFHelperSketchTranslate(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=2, dy=3, dtheta=0)
    #     p = PoseStamped()
    #     p.header.frame_id = "base_link"
    #     transformed_p = self.tf_helper.tf_listener.transformPose("odom", p)
    #
    #     self.assertEqual(transformed_p.header.frame_id, "odom")
    #     self.assertEqual(transformed_p.pose.position.x, 2)
    #     self.assertEqual(transformed_p.pose.position.y, 3)
    #     self.assertEqual(transformed_p.pose.orientation.w, 1)
    #     self.assertEqual(transformed_p.pose.orientation.z, 0)
    #
    #
    # def testNoChangeInMotion(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=0, dy=0, dtheta=0)
    #     self.assertEqual((0, 0, 0), self.motion_model.get_change_in_motion(self.tf_helper))
    #
    # def testChangeInMotionTranslation(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=2, dy=3.5, dtheta=0)
    #     self.assertEqual((2, 3.5, 0), self.motion_model.get_change_in_motion(self.tf_helper))
    #
    # def testChangeInMotionRotation(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=0, dy=0, dtheta=radians(67))
    #     self.assertEqual((0, 0, radians(67)), self.motion_model.get_change_in_motion(self.tf_helper))
    #
    # def testHasNotMovedEnough(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=1, dy=0.5, dtheta=radians(35))
    #     self.assertFalse(self.motion_model.has_moved_enough(self.tf_helper, 3, 40))
    #
    # def testHasTurnedEnough(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=1, dy=0.5, dtheta=radians(75))
    #     self.assertTrue(self.motion_model.has_moved_enough(self.tf_helper, 3, 40))
    #
    # def testHasMovedEnough(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=5, dy=0.5, dtheta=radians(5))
    #     self.assertTrue(self.motion_model.has_moved_enough(self.tf_helper, 3, 40))
    #
    def testPropagateParticleNoRotationOneParticleAtOrigin(self):
        self.tf_helper.tf_listener.setNewTransform(dx=5, dy=0.5, dtheta=0)
        p_list = [Particle(x=0, y=0, theta=0)]
        correct_list = [Particle(x=5, y=0.5, theta=0)]
        self.motion_model.propagate(p_list, self.tf_helper)
        self.assertEqual(p_list, correct_list)

    def testPropagateParticleOnlyRotationParticles(self):
        self.tf_helper.tf_listener.setNewTransform(dx=0, dy=0, dtheta=radians(42))
        p_list = [Particle(x=0, y=0, theta=0), Particle(x=0.4, y=5, theta=354), Particle(x=5, y=-100, theta=42)]
        correct_list = [Particle(x=0, y=0, theta=42),Particle(x=0.4, y=5, theta=36), Particle(x=5, y=-100, theta=84)]
        self.motion_model.propagate(p_list, self.tf_helper)
        self.assertEqual(p_list, correct_list)

    def testPropagateParticleParticlesAtOriginOnlyTranslation(self):
        self.tf_helper.tf_listener.setNewTransform(dx=1, dy=1, dtheta=0)
        p_list = [Particle(x=0, y=0, theta=0),Particle(x=0, y=0, theta=90), Particle(x=0, y=0, theta=45)]
        correct_list = [Particle(x=1, y=1, theta=0),Particle(x=1, y=-1, theta=90), Particle(x=0.5, y=0.5, theta=45)]
        self.motion_model.propagate(p_list, self.tf_helper)
        self.assertEqual(p_list, correct_list)

    # def testPropagateParticleNoRotationParticles(self):
    #     self.tf_helper.tf_listener.setNewTransform(dx=5, dy=0.5, dtheta=0)
    #     p_list = [Particle(x=1, y=0, theta=0),Particle(x=-0.4, y=8, theta=1), Particle(x=-100, y=-945, theta=250)]
    #     correct_list = [Particle(x=6, y=0.5, theta=0),Particle(x=4.6, y=8.5, theta=1), Particle(x=-95, y=-944.5, theta=250)]
    #     self.motion_model.propagate(p_list, self.tf_helper)
    #     self.assertEqual(p_list, correct_list)




if __name__ == '__main__':
    unittest.main()
