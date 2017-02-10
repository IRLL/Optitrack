import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from optitrack_utils import *

import tf
import math

def main():
    # Initialize ros node. Make sure to do this first since the OptitrackUtils
    # class does not do this for you.
    rospy.init_node('OptitrackUtils')

    # Initialize OptitrackUtils with list of rigid body names to get. These
    # names must match the rigid body name in motive.
    opti_utils = OptitrackUtils(["ArDroneA"])

    # Grab the rigid body object
    ArDroneA_rb = opti_utils.get_rigid_body("ArDroneA")

    vel_pub = rospy.Publisher("ArDroneA/linear_velocity", Vector3, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #rospy.loginfo("subscribing to topic: /" + self.vrpndatatopic + "/" + r + "/pose")
        rospy.loginfo("ArdroneA position:\n" + str(ArDroneA_rb.get_position_msg()))

        rospy.loginfo("ArDroneA Linear Velocity:\n" + str(ArDroneA_rb.get_linear_velocity_msg()))

        rospy.loginfo("Publishing linear velocity!\n")
        vel_pub.publish(ArDroneA_rb.get_linear_velocity_msg())

        rate.sleep()

main()
