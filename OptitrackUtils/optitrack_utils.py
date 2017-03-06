import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

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
    def __init__(self, rigid_body_names, republish_pose = False,
            republish_twist = False, republish_rate = 0):
        """
        Subscribes to a ROS topic published by vrpn_client_ros for each
        inputted rigid body name and initializes a RigidBody instance for each.

        Can also republish pose and twist messages at a custom rate for each
        RigidBody.

        Parameters
        ----------
        rigid_body_names : list of str
        republish_pose : bool
        republish_twist : bool
        republish_rate : float
        """

        self.subscribers = []
        self.pose_publishers = {}
        self.twist_publishers = {}
        self.rigid_body_names = rigid_body_names
        self.vrpndatatopic = 'vrpn_client_node'

        self.republish_pose = republish_pose
        self.republish_twist = republish_twist
        self.republish_rate = republish_rate
        self.next_republish_time = rospy.Time.now().to_sec()

        self.rigid_bodies = {}
        for name in self.rigid_body_names:
            rb = RigidBody(name)
            self.rigid_bodies[name] = rb

        self._setup_subscribers()
        self._setup_publishers()

    def _pose_callback(self, pose, name):
        self.rigid_bodies[name].input_pose_msg(pose)

        if rospy.Time.now().to_sec() > self.next_republish_time:
            if self.republish_pose:
                self.pose_publishers[name].publish(self.rigid_bodies[name].get_pose_msg())
            if self.republish_twist:
                self.twist_publishers[name].publish(self.rigid_bodies[name].get_twist_msg())

            # self.next_republish_time is unchanged if republish rate is 0,
            # causing pose callback republish every time it is called.
            if self.republish_rate != 0:
                self.next_republish_time = rospy.Time.now().to_sec() + (1.0 /
                        self.republish_rate)

    def _setup_subscribers(self):
        for idx, r in enumerate(self.rigid_body_names):
            rospy.loginfo("subscribing to topic: /" + self.vrpndatatopic + "/" + r + "/pose")
            sub = rospy.Subscriber(self.vrpndatatopic + "/" + r + "/pose", PoseStamped, self._pose_callback, (r))
            self.subscribers.append(sub)

    def _setup_publishers(self):
        for idx, r in enumerate(self.rigid_body_names):
            if self.republish_pose:
                rospy.loginfo("publishing to topic: /" + r + "/" + r + "/pose")
                pub = rospy.Publisher(r + "/pose", PoseStamped, queue_size=1)
                self.pose_publishers[r] = pub

            if self.republish_twist:
                rospy.loginfo("publishing to topic: /" + r + "/" + r + "/twist")
                pub = rospy.Publisher(r + "/twist", TwistStamped, queue_size=1)
                self.twist_publishers[r] = pub

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
