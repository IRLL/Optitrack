import argparse
import copy
import math
import rospy
import signal
import sys
import tf

from optitrack_utilities.safety_controllers import ArDroneSafetyController

rospy.init_node("turtlebot_safety_controller")
controller = ArDroneSafetyController("PathMarker")
controller.set_safety_bounds("sphere", [0, 0, 0.5], [0.7])
controller.run()
