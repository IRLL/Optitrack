#!/usr/bin/env python

import argparse
import rospy
import signal
import sys

from optitrack_utilities.optitrack_bridge import OptitrackBridge
from optitrack_utilities.engr120 import ENGR120Grader
from optitrack_utilities.path import PathPlotter, Path2D
from optitrack_utilities.path.path_utilities import *

global base_path
global robot_path
global plotter
global args

def handler(signum, frame):
    global base_path
    global robot_path
    global plotter
    global args

    grader = ENGR120Grader(base_path, robot_path, args.student_name)
    grader.grade()

    if args.graph_output_filename:
        plotter.save(args.graph_output_filename)

    rospy.signal_shutdown("exit")
    sys.exit(0)

def main():
    global base_path
    global robot_path
    global plotter
    global args

    rospy.init_node('engr120_grader')

    signal.signal(signal.SIGINT, handler)

    robot_path = Path2D()
    base_path = Path2D()

    robot_path.set_origin_name(args.origin_name)
    base_path.open_path(args.path_filename)

    plotter = PathPlotter()

    opti_utils = OptitrackBridge([args.robot_name])
    robot = opti_utils.get_rigid_body(args.robot_name)

    r = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        if robot.has_received_pose():
            pos = robot.get_position()
            robot_path.add_point(pos[0:2])

            closest_on_path, distance = get_closest_point_and_distance_from_path(robot_path[-1], base_path)

            plotter.input_base_path(base_path)
            plotter.input_robot_path(robot_path)
            plotter.input_closest_point_on_path(closest_on_path)

            plotter.draw()

        r.sleep()

if __name__ == "__main__":
    global args

    args = argparse.ArgumentParser(description="Grade ENGR120")

    args.add_argument("robot_name", type=str, help="Name of robot rigid body.")
    args.add_argument("origin_name", type=str, help="Name of rigid body path is placed on.")
    args.add_argument("path_filename", type=str, help="Name of path file to open.")
    args.add_argument("--student-name", dest="student_name", type=str, help="Name of student or team.")
    args.add_argument("--graph-output-filename", dest="graph_output_filename", type=str, help="Name of png file to save graph of run to. If student name is defined it will be prepended to graph output name.")

    args = args.parse_args()

    if args.graph_output_filename:
        if args.graph_output_filename[-4:] != ".png":
            args.graph_output_filename += ".png"

        if args.student_name:
            args.graph_output_filename = args.student_name + "_" + args.graph_output_filename

    main()
