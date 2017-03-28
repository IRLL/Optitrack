#!/usr/bin/env python

import argparse
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

from utilities.optitrack_utilities import *
from utilities.path2d import Path2D
from utilities.path_utilities import distance_from_path

sys.setrecursionlimit(100)

def main(args):
    rospy.init_node('define_path')

    if args.path_origin:
        opti_utils = OptitrackUtilities([args.marker_name, args.path_origin])
        path_origin_rb = opti_utils.get_rigid_body(args.path_origin)
        marker = opti_utils.get_rigid_body(args.marker_name)
        path = Path2D(max_segment_length = args.max_segment_length, origin_rb = path_origin_rb)

    else:
        opti_utils = OptitrackUtilities([args.marker_name])
        marker = opti_utils.get_rigid_body(args.marker_name)
        path = Path2D(max_segment_length = args.max_segment_length)

    while not rospy.is_shutdown():
        command_str = "[V]: View Location\n[S]: Save Point\n[Q]: Quit and save path\n[R]: Reset\n[X]: Quit without saving\n[D]: Plot Path\n>> "
        in_str = raw_input(command_str)

        if len(in_str) == 0:
            continue

        cmd = in_str[0].lower()

        if cmd[0] == 'v':
            print marker.get_position()[0:2]

        elif cmd[0] == 's':
            ar_pos = marker.get_position()
            path.add_point([ar_pos[0], ar_pos[1]])
            print path

        elif cmd[0] == 'q':
            print "Saved Path: "
            path.save_path(parser.path_filename)
            print path
            rospy.signal_shutdown("Path Saved!")

        elif cmd[0] == 'r':
            path = Path2D(parser.max_segment_length)

        elif cmd[0] == 'x':
            rospy.signal_shutdown("Path Saved!")

        elif cmd[0] == 'd':
            path.draw_base_path()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates a path")

    parser.add_argument("--marker-name", dest="marker_name", type=str,
        default="PathMarker", help="name of rigid body used as path marker")
    parser.add_argument("--path-filename", dest="path_filename", type=str,
        default="paths/new_path.path", help="name of path file to create")
    parser.add_argument("--max-segment-length", dest="max_segment_length",
        type=float, default=0.2, help="max length of any segment in path")
    parser.add_argument("--path-origin", dest="path_origin", type=str,
        help="name of rigid body path is placed on")

    parser = parser.parse_args()

    print parser

    main(parser)
