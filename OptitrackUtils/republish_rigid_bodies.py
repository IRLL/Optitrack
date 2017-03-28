#!/usr/bin/env python

import argparse
import math
import os
import rospy
import tf

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from utilities.optitrack_utilities import *

def main(parser):
    rospy.init_node("republish_rigid_bodies")

    opti_utils = OptitrackUtilities(parser.names, parser.republish_pose,
            parser.republish_twist, parser.republish_rate)

    rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Republish pose and twist for \
            rigid bodies from vrpn_client_ros.")

    parser.add_argument("names", type=str, metavar="names", nargs="+", help="names of rigid \
            bodies as defined in Motive")
    parser.add_argument("--republish_pose", dest="republish_pose",
            default=True, type=bool, help="whether to republish pose msgs")
    parser.add_argument("--republish_twist", dest="republish_twist", type=bool,
            default=True, help="whether to republish twist msgs")
    parser.add_argument("--republish_rate", dest="republish_rate", type=float,
            default=0, help="rate to republish msgs at. if equal to 0: \
            republish as fast as possible")
    parser = parser.parse_args()

    main(parser)
