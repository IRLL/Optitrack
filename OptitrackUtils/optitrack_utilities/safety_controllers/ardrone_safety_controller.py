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

class ArDroneSafetyController:
    def __init__(self, ardrone_name):
        self.takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty,
                queue_size=1)
        self.land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=1)
        self.reset_pub = rospy.Publisher("ardrone/reset", Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("safety/ardrone/takeoff", Empty, self.takeoff_callback)
        rospy.Subscriber("safety/ardrone/land", Empty, self.land_callback)
        rospy.Subscriber("safety/ardrone/reset", Empty, self.reset_callback)
        rospy.Subscriber("safety/cmd_vel", Twist, self.cmd_vel_callback)

        opti_utils = OptitrackBridge(ardrone_name)
        self.ardrone_rb = opti_utils.get_rigid_body(ardrone_name)

        self.boundary = Boundary("none", [0,0,0], [0,0,0])

    def set_safety_bounds(self, shape, shape_center, shape_dims):
        if shape == "none":
            self.boundary = Boundary("none", [0,0,0], [0,0,0])
        elif shape == "box":
            self.boundary = BoxBoundary(shape_center, shape_dims)
        elif shape == "sphere":
            self.boundary = SphereBoundary(shape_center, shape_dims)
        else:
            print "Unknown shape: " + shape

    def takeoff_callback(self, msg):
        print msg
        self.takeoff_pub.publish(msg)

    def land_callback(self, msg):
        print msg
        self.land_pub.publish(msg)

    def reset_callback(self, msg):
        print msg
        self.reset_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        if self.ardrone_rb.has_received_pose():
            if self.boundary.within_bounds(self.ardrone_rb.get_position()):
                print "Within bounds. Sending msg."
                self.cmd_vel_pub.publish(msg)
            else:
                print "Out of bounds."
        else:
            print "No ardrone pose received."

        print

    def run(self):
        rospy.spin()
