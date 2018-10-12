import rospy
from ParticleDistribution import ParticleDistribution
from Particle import Particle
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray
import random

class ResamplingTest(object):
    def __init__(self):
        rospy.init_node("resampling_test")
        self.p_distrib = ParticleDistribution(num_particles=50)
        self.particle_pose_pub = rospy.Publisher('/particle_pose_array', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.counter = 0

    def init_particles(self):
        self.p_distrib.particle_list = []
        # generate initial list of hypothesis (particles)
        for i in range(self.p_distrib.num_particles):
            # Find a random valid point on the map
            x = random.uniform(-2, 2)
            y = random.uniform(-2, 2)
            theta = random.randint(0, 361)
            weight = abs(random.gauss(0.3, 0.2))
            # Add new particle to list
            self.p_distrib.particle_list.append(Particle(x=x, y=y, theta=theta, weight=weight))
        # Normalize weights
        self.p_distrib.normalize_weights()

    def display_distrib(self):
        pose_array = self.p_distrib.get_particle_pose_array()
        pose_array.header.stamp = rospy.Time.now()
        self.particle_pose_pub.publish(pose_array)

        marker_arr = MarkerArray()
        for p in self.p_distrib.particle_list:
            my_marker = Marker()
            my_marker.header.stamp = rospy.Time.now()
            my_marker.header.frame_id = "map"

            my_marker.color.a = 0.7
            my_marker.color.r = 1.0 - p.weight / max(self.p_distrib.get_weights())
            my_marker.color.b = p.weight / max(self.p_distrib.get_weights()) + 0.1

            my_marker.scale.x = p.weight + 0.1
            my_marker.scale.y = p.weight + 0.1
            my_marker.scale.z = p.weight + 0.1

            my_marker.type = Marker.SPHERE
            my_marker.id = self.counter
            self.counter += 1

            my_marker.pose.position.x = p.x
            my_marker.pose.position.y = p.y
            my_marker.lifetime = rospy.Time(2)
            marker_arr.markers.append(my_marker)

        self.marker_pub.publish(marker_arr)

    def run(self):
        while not rospy.is_shutdown():
            r = rospy.Rate(0.5)
            # Display initial particles
            self.init_particles()
            self.display_distrib()

            r.sleep()

            # Resample particles, give them rand rotations and display
            self.p_distrib.resample()
            self.p_distrib.normalize_weights()
            for p in self.p_distrib.particle_list:
                p.theta = random.randint(0, 361)
            self.display_distrib()

            r.sleep()

            self.p_distrib.particle_list = []
            self.display_distrib()
            r.sleep()

if __name__ == "__main__":
    rt = ResamplingTest()
    rt.run()
