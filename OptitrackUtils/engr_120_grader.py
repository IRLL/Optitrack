#!/usr/bin/env python

import rospy
import sys

from utilities.optitrack_utilities import *
from utilities.path2d import Path2D

errors = []

path = Path2D(1.0)

path.open_path("paths/table_many_points.path")

print path

path.draw_base_path()

rospy.init_node('engr_120_grader')

opti_utils = OptitrackUtilities(["PathMarker"])
marker = opti_utils.get_rigid_body("PathMarker")

while not rospy.is_shutdown():
    if marker.has_new_pose():
        pos = (marker.get_position()[0], marker.get_position()[1])
        #marker.set_new_pose_false()

        print "pos: "
        print pos
        print "path.get_closest_path_point(pos): "
        print path.get_closest_path_point(pos)
        print "path.find_distance_from_path(pos): "
        distance_from = path.find_distance_from_path(pos)
        print distance_from
        print

        errors.append(distance_from)

        path.draw_robot_location(pos)
