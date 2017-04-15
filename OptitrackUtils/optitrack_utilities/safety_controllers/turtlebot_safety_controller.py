import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from ..optitrack_bridge.optitrack_bridge import *
from boundary import *

# Can be class
    # Params:
        # Topics: safety (type must be configurable)
# Subscribe to safety/ardrone_controller
# Start OptitrackBridge
# Wait until ^ initialized
# On safety/ardrone_controller callback:
    #
    #
    #
# Pubs:
    # cmd_vel or whatev

class TurtlebotSafetyController:
    def __init__(self, turtlebot_name):
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("safety/cmd_vel", Twist, self.cmd_vel_callback)

        opti_utils = OptitrackBridge(turtlebot_name)
        self.turtlebot_rb = opti_utils.get_rigid_body(turtlebot_name)

        self.boundary = Boundary("none", [0,0,0], [0,0,0])

    def set_safety_bounds(self, shape, shape_center, shape_dims):
        if shape == "none":
            self.boundary = Boundary("none", [0,0,0], [0,0,0])
        elif shape == "box":
            self.boundary = BoxBoundary(shape_center, shape_dims)
        elif shape == "sphere":
            self.boundary = SphereBoundary(shape_center, shape_dims)
        else:
            raise ValueError("Unknown shape: " + shape)

    def cmd_vel_callback(self, msg):
        if self.turtlebot_rb.has_received_pose():
            if self.boundary.within_bounds(self.turtlebot_rb.get_position()):
                print "Within bounds. Sending msg."
                self.cmd_vel_pub.publish(msg)
            else:
                print "Out of bounds."
        else:
            print "No turtlebot pose received."

        print

    def run(self):
        rospy.spin()
