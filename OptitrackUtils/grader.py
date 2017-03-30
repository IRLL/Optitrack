#!/usr/bin/env python

import argparse
import rospy
import signal
import sys

from utilities.path2d import *
from utilities.engr120_grader import *


global base_path
global robot_path
global plotter
global parser

def handler(signum, frame):
    global plotter
    global parser

    g = ENGR120_Grader(base_path.get_path(), robot_path.get_path())
    g.grade()

    plotter.save(parser.results_filename)

    rospy.signal_shutdown("exit")
    sys.exit(0)

def main():
    global base_path
    global robot_path
    global plotter
    global parser

    rospy.init_node('open_path')

    signal.signal(signal.SIGINT, handler)

    base_path = Path2D(filename=parser.path_filename)
    robot_path = Path2D(origin_name=parser.origin_name)

    plotter = PathPlotter()

    opti_utils = OptitrackUtilities([parser.robot_name])
    robot = opti_utils.get_rigid_body(parser.robot_name)

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        if robot.has_received_message():
            pos = robot.get_position()
            robot_path.add_point(pos[0:2])

            closest_on_path, distance = get_closest_point_and_distance_from_path(robot_path.get_path()[-1], base_path.get_path())

            plotter.input_base_path(base_path.get_path())
            plotter.input_robot_path(robot_path.get_path())
            plotter.input_closest_point_on_path(closest_on_path)

            plotter.draw()

        r.sleep()

if __name__ == "__main__":
    global parser
    parser = argparse.ArgumentParser(description="Grade ENGR120")

    parser.add_argument("--robot-name", dest="robot_name", type=str,
        default="PathMarker", help="name of robot rigid body")
    parser.add_argument("--path-filename", dest="path_filename", type=str,
        default="paths/table.path", help="name of path file to open")
    parser.add_argument("--path-results-filename", dest="results_filename", type=str,
        default="plots/new_plot.png", help="name of file to save grade to")
    parser.add_argument("--origin-name", dest="origin_name", type=str,
        default="TableCenter", help="name of rigid body path is placed on")

    parser = parser.parse_args()

    main()
