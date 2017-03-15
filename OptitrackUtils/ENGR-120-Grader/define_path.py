import math
import rospy
import signal
import sys
import tf

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from path_utils import distance_from_path
from path_2d import Path2D

sys.path.append("../")
from optitrack_utils import *

# TODO:
# Get rigidbody offset
# Draw functions
# Path Lengths

def main():
    rospy.init_node('OptitrackUtils')

    opti_utils = OptitrackUtils(["PathMarker"], True, True, 2)
    marker = opti_utils.get_rigid_body("PathMarker")

    # path = Path2D(1.0)
    # print "Opening Path mypath.path: "
    # path.open_path("mypath.path")
    # print path
    path = Path2D(0.1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        command_str = "[V]: View Location\n[S]: Save Point\n[Q]: Quit and save path\n[R]: Reset\n[X]: Quit without saving\n"
        in_str = raw_input(command_str)

        if len(in_str) == 0:
            continue

        cmd = in_str[0].lower()

        if cmd[0] == 'v':
            print marker.get_position()[0:2]

        elif cmd[0] == 's':
            ar_pos = marker.get_position()
            path.add_point((ar_pos[0], ar_pos[1]))
            print path

        elif cmd[0] == 'q':
            print "Saved Path: "
            path.save_path("mypath.path")
            print path
            rospy.signal_shutdown("Path Saved!")

        elif cmd[0] == 'r':
            path = Path2D(1.0)

        elif cmd[0] == 'x':
            rospy.signal_shutdown("Path Saved!")

if __name__ == "__main__":
    main()
