#!/usr/bin/env python

import argparse
import copy
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
from utilities.path_creator import PathCreator
from utilities.path_plotter import PathPlotter
from utilities.path_utilities import get_closest_point_and_distance_from_path

def main(args):
    rospy.init_node('define_path')

    if args.origin_name:
        opti_utils = OptitrackUtilities([args.marker_name, args.origin_name])
        origin_rigidbody = opti_utils.get_rigid_body(args.origin_name)
        marker = opti_utils.get_rigid_body(args.marker_name)
        path = PathCreator(max_segment_length = args.max_segment_length, origin_rb = origin_rigidbody)

    else:
        opti_utils = OptitrackUtilities([args.marker_name])
        marker = opti_utils.get_rigid_body(args.marker_name)
        path = PathCreator(max_segment_length = args.max_segment_length)

    plotter = PathPlotter()
    draw_path = True
    while not rospy.is_shutdown():
        command_str = "[V]: View Location\n[S]: Save Point\n[Q]: Quit and Save Path\n[R]: Reset\n[X]: Quit Without Saving\n[D]: Toggle Path Plotting\n>> "
        in_str = raw_input(command_str)

        if len(in_str) == 0:
            continue

        cmd = in_str[0].lower()

        print

        if cmd[0] == 'v':
            if args.origin_name:
                print args.origin_name + ": " + str(origin_rigidbody.get_position()[0:2])
            print args.marker_name + ": " + str(marker.get_position()[0:2])

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
            path = PathCreator(parser.max_segment_length)

        elif cmd[0] == 'x':
            rospy.signal_shutdown("Path Saved!")

        elif cmd[0] == 'd':
            draw_path = True if not draw_path else False

        if draw_path:
            plotter.input_base_path(path.get_path())
            plotter.draw()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates a path")

    parser.add_argument("--marker-name", dest="marker_name", type=str,
        default="PathMarker", help="name of rigid body used as path marker")
    parser.add_argument("--path-filename", dest="path_filename", type=str,
        default="paths/new_path.path", help="name of path file to create")
    parser.add_argument("--max-segment-length", dest="max_segment_length",
        type=float, default=0.2, help="max length of any segment in path")
    parser.add_argument("--origin-name", dest="origin_name", type=str,
        help="name of rigid body path is placed on")

    parser = parser.parse_args()

    main(parser)
