import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from enum import Enum
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header

class ROSDtype(Enum):
    FLOAT = Float64
    FLOAT_ARRAY = Float64MultiArray
    JOINT_STATE = JointState
    WRENCH = WrenchStamped
    FORCE = WrenchStamped
    IMU = Imu
    CLOCK  = Clock
    
class ROSClock(Clock):
    def __init__(self, time):
        super().__init__()
        self.clock = rospy.Time.from_sec(time)
        # header=Header(stamp=rospy.Time.from_sec(time))
        # self = Clock()

class RobotJointState(JointState):
    def __init__(self, name, pos, vel, tor):
        super().__init__()
        self.name = name
        self.position = pos
        self.velocity = vel
        self.effort = tor

class ImuData(Imu):
    def __init__(self, quat, ang_vel, lin_acc):
        super().__init__()
        self.orientation.x = quat[0]
        self.orientation.y = quat[1]
        self.orientation.z = quat[2]
        self.orientation.w = quat[3]
        self.angular_velocity.x = ang_vel[0]
        self.angular_velocity.y = ang_vel[1]
        self.angular_velocity.z = ang_vel[2]
        self.linear_acceleration.x = lin_acc[0]
        self.linear_acceleration.y = lin_acc[1]
        self.linear_acceleration.z = lin_acc[2]

def data_to_ros_msg(data, dtype:ROSDtype, ros_time):
    ros_msg = None
    if dtype == ROSDtype.FLOAT_ARRAY:
        data = np.array(data)
        ros_msg = Float64MultiArray()
        ros_msg.data = data

    elif dtype == ROSDtype.JOINT_STATE:
        ros_msg = JointState()
        ros_msg = data
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)

    elif dtype == ROSDtype.FORCE:
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = 0
        ros_msg.wrench.torque.y = 0
        ros_msg.wrench.torque.z = 0
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)

    elif dtype == ROSDtype.WRENCH:
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = data[3]
        ros_msg.wrench.torque.y = data[4]
        ros_msg.wrench.torque.z = data[5]
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)

    elif dtype == ROSDtype.IMU:
        ros_msg = JointState()
        ros_msg = data
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)
    
    elif dtype == ROSDtype.CLOCK:
        ros_msg = data
    return ros_msg