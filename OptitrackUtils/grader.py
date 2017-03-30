#!/usr/bin/env python

import rospy
import signal
import sys

from utilities.path2d import *
from utilities.engr120_grader import *

global base_path
global robot_path

def handler(signum, frame):
    g = ENGR120_Grader(base_path.get_path(), robot_path.get_path())
    g.grade()

    rospy.signal_shutdown("exit")
    sys.exit(0)

def main():
    global base_path
    global robot_path

    rospy.init_node('open_path')

    signal.signal(signal.SIGINT, handler)

    base_path = Path2D(filename="paths/white_table.path")
    robot_path = Path2D(origin_name="TableCenter")

    opti_utils = OptitrackUtilities(["PathMarker"])
    robot = opti_utils.get_rigid_body("PathMarker")

    plotter = PathPlotter()
    r = rospy.Rate(5.0)
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

main()
