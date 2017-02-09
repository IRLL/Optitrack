import rospy
from geometry_msgs.msg import PoseStamped

from rigid_body import *

class OptitrackUtils:
    """
    Used in conjunction with vrpn_client_ros to receive and store rigid body
    data that is being published.

    Please make sure to initialize a ros node before using this class.

    Attributes
    ----------
    rigid_body_names : list of str
    rigid_bodies : list of RigidBody
    """
    def __init__(self, rigid_body_names):
        """
        Subscribes to a ROS topic published by vrpn_client_ros for each
        inputted rigid body name and initializes a RigidBody instance for each.

        Parameters
        ----------
        rigid_body_names : list of str
        """

        self.subscribers = []
        self.rigid_body_names = rigid_body_names
        self.vrpndatatopic = 'vrpn_client_node'

        self.rigid_bodies = {}
        for name in self.rigid_body_names:
            rb = RigidBody(name)
            self.rigid_bodies[name] = rb

        self._setup_subscribers()

    def _pose_callback(self, pose, name):
        self.rigid_bodies[name].input_pose_msg(pose)

    def _setup_subscribers(self):
        for idx, r in enumerate(self.rigid_body_names):
            rospy.loginfo("subscribing to topic: /" + self.vrpndatatopic + "/" + r + "/pose")
            sub = rospy.Subscriber(self.vrpndatatopic + "/" + r + "/pose", PoseStamped, self._pose_callback, (r))
            self.subscribers.append(sub)

    def get_rigid_body(self, name):
        """
        Returns RigidBody of specified name

        Parameters
        ----------
        name : str

        Returns
        -------
        rigid_body : RigidBody
        """

        return self.rigid_bodies[name]

"""
# Necessary?
    def get_rigid_body_pose(self, name):
        return self.rigid_bodies[name].get_pose_msg()

    def get_rigid_body_position(self, name):
        return self.rigid_bodies[name].get_position()

    def get_rigid_body_orientation(self, name):
        return self.rigid_bodies[name].get_orientation()

    def get_rigid_body_linear_velocity(self, name):
        return self.rigid_bodies[name].get_linear_velocity()
"""
