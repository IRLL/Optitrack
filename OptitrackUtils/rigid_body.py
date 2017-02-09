import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from math import fabs
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
from collections import deque

# Internally uses tf notation for vectors and quaternions, that is, they are
# array like. Vectors are [x,y,z] and quaternions are [x,y,z,w]
# Input is in rosmsg

# TODO: Linear velocity not correct.
# Linear velocity is much more correct when vrpn_client_ros uses server time
# rather than client time
# Linear velocity can be made much smoother with a windowed running average as
# well

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
        NOT IMPLEMENTED YET
    """
    def __init__(self, name=""):
        """
        Convenience function for creating a 3 dimensional vector with an array-like
        format.

        Parameters
        ----------
        name : string, optional
            Name of rigid body
        """

        self.name = name
        self.last_pose_msg = PoseStamped()
        self.pose_msg = PoseStamped()
        self.position = Vector3Array(0, 0, 0)
        self.orientation = QuaternionArray(0, 0, 0, 1)

        # TODO: Turn into full twist message
        self.position_diff_queue = deque(maxlen=5)
        self.linear_velocity = Vector3Array(0, 0, 0)
        self.angular_velocity = Vector3Array(0, 0, 0)

        self.last_timestamp = rospy.Time.now()

    def input_pose_msg(self, pose):
        """
        Sets the pose (position and orientation) of the rigid body.

        Parameters
        ----------
        pose : geometry_msgs.msgs.Pose
            ROS Pose msg of rigid body
        """
        self.pose_msg = pose

        self.position = Vector3Array(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z)

        self.orientation = QuaternionArray(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)

        self._calculate_linear_velocity()

        self.last_timestamp = self.pose_msg.header.stamp
        self.last_pose_msg = self.pose_msg

    def _calculate_linear_velocity(self):
        dt = (self.pose_msg.header.stamp - self.last_timestamp).to_sec()
        pos_diff = Vector3Array(
                self.pose_msg.pose.position.x - self.last_pose_msg.pose.position.x,
                self.pose_msg.pose.position.y - self.last_pose_msg.pose.position.y,
                self.pose_msg.pose.position.z - self.last_pose_msg.pose.position.z)

        self.position_diff_queue.append((pos_diff, dt))

        self.linear_velocity = Vector3Array(0, 0, 0)
        for item in self.position_diff_queue:
            self.linear_velocity[0] += item[0][0] / item[1]
            self.linear_velocity[1] += item[0][1] / item[1]
            self.linear_velocity[2] += item[0][2] / item[1]

        self.linear_velocity[0] /= len(self.position_diff_queue)
        self.linear_velocity[1] /= len(self.position_diff_queue)
        self.linear_velocity[2] /= len(self.position_diff_queue)

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

    def get_position_error(self, desired_pos):
        """
        Finds difference between rigid body position and inputted position.

        Parameters
        ----------
        desired_position : Vector3Array
            Array of [x, y, z] coordinates of desired position

        Returns
        -------
        error: Vector3Array
            Difference between rigid bodies position and inputted position
        """
        return [self.position.x - desired_pos[0],
                self.position.y - desired_pos[1],
                self.position.z - desired_pos[2]]

    def get_orientation_error(self, desired_orientation):
        """
        Finds difference between rigid body position and inputted position.

        Utilizes the following formula:

        error = desired_orientation * inverse(current_orientation)

        where error, desired_orientation, and current_orientation are
        quaternions, and inverse() denotes the quaternion inverse.

        Parameters
        ----------
        desired_orientation : QuaternionArray
            Array of [x, y, z, w] of desired orientation

        Returns
        -------
        error: QuaternionArray
            Difference between rigid bodies orientation and inputted orientation
        """

        inverted = QuaternionArray(
                -self.orientation[0],
                -self.orientation[1],
                -self.orientation[2],
                self.orientation[3])

        # quaternion_multiply uses tf representation so quaternions are
        # represented as a vector of [x, y, z, w]
        return quaternion_multiply(desired_orientation, inverted)


    def get_pose_msg(self):
        """
        Returns last received pose msg.

        Returns
        -------
        pose : geometry_msgs.msgs.Pose
        """
        return self.pose_msg

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
