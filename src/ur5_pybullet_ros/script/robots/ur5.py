from robots.robot_base import RobotBase
import os
import pybullet as p
import gin
import math
from ros_wrapper.ros_wrapper import RosWrapper
from ros_wrapper.ros_msg import ROSDtype, RobotJointState

ROS_SET_ANGLE_TOPIC = "set_angle"
ROS_JOINT_STATES_TOPIC = "joint_states"
ROS_JOINT_ANGLE_TOPIC = "joint_angles"

@gin.configurable
class UR5(RobotBase):
    def __init__(self, urdf_file, base_pos, base_ori, inital_angle, gripper_range, arm_joint, eef_joint):
        self.name = "UR5"
        urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../urdf/" + urdf_file
        super().__init__(self.name, urdf_file, base_pos, base_ori, inital_angle, gripper_range, arm_joint, eef_joint)
        self.reset()
        self.time = 0
        self.set_angle = [inital_angle] # we use list to make it a mutable variable, so the callback of ros can change this value naturely
        self.joint_info_all = {}
        self.joint_arm_info = {}
        self.init_ros_interface()

    def init_ros_interface(self):
        self.ros_wrapper = RosWrapper("ur5_pybullet")
        self.ros_wrapper.add_subscriber(ROS_SET_ANGLE_TOPIC, ROSDtype.FLOAT_ARRAY, self.set_angle)
        self.ros_wrapper.add_publisher(ROS_JOINT_STATES_TOPIC, ROSDtype.JOINT_STATE, False)
        self.ros_wrapper.add_publisher(ROS_JOINT_ANGLE_TOPIC, ROSDtype.FLOAT_ARRAY)

    def post_control(self):
        self.pub_ros_info()

    def pre_control(self):
        self.time = self.ros_wrapper.ros_time
        self.joint_info_all = self.get_rotate_joint_info_all()
        self.joint_arm_info = self.get_joint_obs()

    def pub_ros_info(self):
        joint_state = RobotJointState(self.rotate_joint_names, self.joint_info_all["positions"], self.joint_info_all["velocities"], self.joint_info_all["torques"])
        self.ros_wrapper.publish_msg(ROS_JOINT_STATES_TOPIC, joint_state)
        # print(joint_info)
        
    def move_gripper(self, open_length):
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation


CONFIG_FILE = (os.path.dirname(os.path.abspath(__file__)) + "/../config/ur5_default.gin")
gin.parse_config_file(CONFIG_FILE)

if __name__ == "__main__":
    p.connect(p.GUI)
    ur5_robot = UR5()
    while True:
        pass