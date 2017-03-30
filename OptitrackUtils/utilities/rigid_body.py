import rospy

from collections import deque
from math import fabs

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_inverse
from tf.transformations import quaternion_multiply

def Vector3Array(x, y, z):
    """
    Convenience function for creating a 3 dimensional vector with an array-like
    format.

    Parameters
    ----------
    x : int or float
    y : int or float
    z : int or float

    Returns
    -------
    vector : Vector3Array
    """
    return [float(x), float(y), float(z)]

def QuaternionArray(x, y, z, w):
    """
    Convenience function for creating a quaternion with an array-like format.

    Parameters
    ----------
    x : int or float
    y : int or float
    z : int or float
    w : int or float

    Returns
    -------
    vector : QuaternionArray
    """
    return [float(x), float(y), float(z), float(w)]

class RigidBody:
    """
    Stores position, orientation, and other information about a named rigid
    body.

    Follows ROS tf standards for storing vector and quaternion data, that is,
    they are stored in an array-like format and can be indexed using [].

    Attributes
    ----------
    name : str
        Rigid body name
    position : Vector3Array
        Three dimensional vector array of position of rigid body
    orientation : QuaternionArray
        Quaternion array of orientation of rigid body
    linear_velocity : Vector3Array
        Three dimensional vector array of linear velocity of rigid body
    angular_velocity : Vector3Array
        Three dimensional vector array of angular velocity of rigid body
    """
    def __init__(self, name=""):
        """
        Initialization for named rigid body

        Parameters
        ----------
        name : string, optional
            Name of rigid body
        """

        self.name = name
        self.pose_received = False

        self.last_pose_msg = PoseStamped()
        self.last_twist_msg = PoseStamped()
        self.pose_msg = PoseStamped()
        self.twist_msg = TwistStamped()
        self.position = Vector3Array(0, 0, 0)
        self.orientation = QuaternionArray(0, 0, 0, 1)

        self.position_diff_queue = deque(maxlen=5)
        self.angular_diff_queue = deque(maxlen=5)
        self.linear_velocity = Vector3Array(0, 0, 0)
        self.angular_velocity = Vector3Array(0, 0, 0)

        self.last_timestamp = rospy.Time.now()
        self.dt = 0.0

    def input_pose_msg(self, pose):
        """
        Sets the pose (position and orientation) of the rigid body.

        Parameters
        ----------
        pose : geometry_msgs.msgs.Pose
            ROS Pose msg of rigid body
        """
        self.pose_msg = pose
        self.pose_received = True

        self.position = Vector3Array(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z)

        self.orientation = QuaternionArray(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)

        self._calculate_dt()
        self._calculate_linear_velocity()
        self._calculate_angular_velocity()

        self.twist_msg.header = self.pose_msg.header
        self.twist_msg.twist.linear = Vector3(*self.linear_velocity)
        self.twist_msg.twist.angular = Vector3(*self.angular_velocity)

        self.last_timestamp = self.pose_msg.header.stamp
        self.last_pose_msg = self.pose_msg
        self.last_twist_msg = self.twist_msg

    def _calculate_dt(self):
        self.dt = (self.pose_msg.header.stamp - self.last_timestamp).to_sec()

    def _calculate_linear_velocity(self):
        pos_diff = Vector3Array(
                self.pose_msg.pose.position.x - self.last_pose_msg.pose.position.x,
                self.pose_msg.pose.position.y - self.last_pose_msg.pose.position.y,
                self.pose_msg.pose.position.z - self.last_pose_msg.pose.position.z)

        self.position_diff_queue.append((pos_diff, self.dt))

        self.linear_velocity = Vector3Array(0, 0, 0)
        for item in self.position_diff_queue:
            if item[1] != 0.0:
                self.linear_velocity[0] += item[0][0] / item[1]
                self.linear_velocity[1] += item[0][1] / item[1]
                self.linear_velocity[2] += item[0][2] / item[1]

        self.linear_velocity[0] /= len(self.position_diff_queue)
        self.linear_velocity[1] /= len(self.position_diff_queue)
        self.linear_velocity[2] /= len(self.position_diff_queue)

    def _calculate_angular_velocity(self):
        previous_orientation = QuaternionArray(
                self.last_pose_msg.pose.orientation.x,
                self.last_pose_msg.pose.orientation.y,
                self.last_pose_msg.pose.orientation.z,
                self.last_pose_msg.pose.orientation.w)

        # Calculate difference in orientation between previous and current position as a quaternion.
        diff_quat = quaternion_multiply(quaternion_inverse(previous_orientation), self.orientation)

        diff_euler = [i for i in euler_from_quaternion(diff_quat)]

        self.angular_diff_queue.append((diff_euler, self.dt))

        self.angular_velocity = Vector3Array(0, 0, 0)
        for item in self.angular_diff_queue:
            if item[1] != 0.0:
                self.angular_velocity[0] += item[0][0] / item[1]
                self.angular_velocity[1] += item[0][1] / item[1]
                self.angular_velocity[2] += item[0][2] / item[1]

        self.angular_velocity[0] /= len(self.angular_diff_queue)
        self.angular_velocity[1] /= len(self.angular_diff_queue)
        self.angular_velocity[2] /= len(self.angular_diff_queue)

    def within_box(self, box_center, box_dimensions):
        """
        Checks whether the position of the rigid body is within an inputted box.

        Parameters
        ----------
        box_center : Vector3Array
            Array of [x, y, z] coordinates of center of the box
        box_dimensions : Vector3Array
            Array of [x, y, z] denoting side lengths of the box

        Returns
        -------
        within : bool
            True if rigid body is within inputted box
        """
        box_dimensions[0] /= 2.0
        box_dimensions[1] /= 2.0
        box_dimensions[2] /= 2.0

        x_width = box_center[0] + box_dimensions[0]
        y_width = box_center[1] + box_dimensions[1]
        z_width = box_center[2] + box_dimensions[2]

        if fabs(self.position.x - box_center[0]) > x_width:
            return False
        if fabs(self.position.y - box_center[1]) > y_width:
            return False
        if fabs(self.position.z - box_center[2]) > z_width:
            return False

        return True

    def has_received_message(self):
        return self.pose_received

    def get_position_difference(self, desired_pos):
        """
        Finds difference between rigid body position and inputted position.

        Parameters
        ----------
        desired_position : Vector3Array
            Array of [x, y, z] coordinates of desired position

        Returns
        -------
        difference: Vector3Array
            Difference between rigid bodies position and inputted position
        """
        return [self.position.x - desired_pos[0],
                self.position.y - desired_pos[1],
                self.position.z - desired_pos[2]]

    def get_orientation_difference(self, desired_orientation):
        """
        Finds difference between rigid body position and inputted position.

        Utilizes the following formula:

        difference = desired_orientation * inverse(current_orientation)

        where difference, desired_orientation, and current_orientation are
        quaternions, and inverse() denotes the quaternion inverse.

        Parameters
        ----------
        desired_orientation : QuaternionArray
            Array of [x, y, z, w] of desired orientation

        Returns
        -------
        difference: QuaternionArray
            Difference between rigid bodies orientation and inputted orientation
        """

        inverted = quaternion_inverse(self.orientation)

        return quaternion_multiply(desired_orientation, inverted)


    def get_pose_msg(self):
        """
        Returns last received pose msg.

        Returns
        -------
        pose : geometry_msgs.msgs.Pose
        """
        return self.pose_msg

    def get_twist_msg(self):
        """
        Returns last received twist msg.

        Returns
        -------
        twist : geometry_msgs.msgs.Twist
        """
        return self.twist_msg

    def get_position(self):
        """
        Returns rigid body position.

        Returns
        -------
        position : Vector3Array
        """
        return self.position

    def get_orientation(self):
        """
        Returns rigid body orientation.

        Returns
        -------
        orientation : QuaternionArray
        """
        return self.orientation

    def get_linear_velocity(self):
        """
        Returns rigid body linear velocity.

        Returns
        -------
        linear_velocity : Vector3Array
        """
        return self.linear_velocity

    def get_angular_velocity(self):
        """
        Returns rigid body angular velocity.

        Returns
        -------
        angular_velocity : Vector3Array
        """
        return self.angular_velocity

    def get_position_msg(self):
        """
        Returns rigid body position as ros msg.

        Returns
        -------
        position : geometry_msgs.msgs.Vector3
        """
        return Point(*self.position)

    def get_orientation_msg(self):
        """
        Returns rigid body orientation as ros msg.

        Returns
        -------
        orientation : geometry_msgs.msgs.Quaternion
        """
        return Quaternion(*self.orientation)

    def get_linear_velocity_msg(self):
        """
        Returns rigid body linear velocity as ros msg.

        Returns
        -------
        linear_velocity : geometry_msgs.msgs.Vector3
        """
        return Vector3(*self.linear_velocity)

    def get_angular_velocity_msg(self):
        """
        Returns rigid body angular velocity as ros msg.

        Returns
        -------
        angular_velocity : geometry_msgs.msgs.Vector3
        """
        return Vector3(*self.angular_velocity)

    def get_dt(self):
        """
        Return dt

        Returns
        -------
        dt : float
        """
        return self.dt
