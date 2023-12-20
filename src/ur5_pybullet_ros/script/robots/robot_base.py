import pybullet as p
from collections import namedtuple


class RobotBase(object):
    """
    The base class for robots
    """

    def __init__(self, name, urdf_file, pos, ori, inital_angle, gripper_range, arm_joint, eef_joint):
        """
        Arguments:
            pos: [x y z]
            ori: [x y z w]

        Attributes:
            id: Int, the ID of the robot
            eef_id: Int, the ID of the End-Effector
            arm_num_dofs: Int, the number of DoFs of the arm
                i.e., the IK for the EE will consider the first `arm_num_dofs` controllable (non-Fixed) joints
            joints: List, a list of joint info
            controllable_joints: List of Ints, IDs for all controllable joints
            arm_controllable_joints: List of Ints, IDs for all controllable joints on the arm (that is, the first `arm_num_dofs` of controllable joints)

            ---
            For null-space IK
            ---
            arm_lower_limits: List, the lower limits for all controllable joints on the arm
            arm_upper_limits: List
            arm_joint_ranges: List
            arm_rest_poses: List, the rest position for all controllable joints on the arm

            gripper_range: List[Min, Max]
        """
        
        self.base_pos = pos
        self.base_ori = ori
        
        self.name = name
        self.urdf_file = urdf_file
        self.id = p.loadURDF(self.urdf_file, self.base_pos, self.base_ori,
                              flags=p.URDF_USE_INERTIA_FROM_FILE + p.URDF_USE_SELF_COLLISION)
        self.arm_rest_poses = inital_angle
        self.arm_num_dofs = len(inital_angle)
        self.gripper_range = gripper_range
        self.arm_joint = arm_joint
        self.eef_joint = eef_joint
        
        self.load()

    def load(self):
        self.__parse_joint_info__()
        self.__post_load__()
        # print(self.joints)


    def __parse_joint_info__(self):
        numJoints = p.getNumJoints(self.id)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.arm_motor_ids = []
        self.joints_name = []
        
        self.rotate_joint_names = []
        self.rotate_joint_id = []
        
        for i in range(numJoints):
            info = p.getJointInfo(self.id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED) and jointName in self.arm_joint
            if controllable:
                p.setJointMotorControl2(self.id, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)
            if self.eef_joint == jointName:
                self.eef_id = jointID
            
            if jointType is not 4:
                self.rotate_joint_names.append(jointName)
                self.rotate_joint_id.append(jointID)
            # print(jointID, jointName)
            
            
            self.joints_name.append(jointName)
        assert hasattr(self, 'eef_id'), "eef_id is not found!"
        for motor_joint in self.arm_joint:
            self.arm_motor_ids.append(self.joints_name.index(motor_joint))

        self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
    
    def __post_load__(self):
        pass

    def reset(self):
        self.reset_arm()
        self.reset_gripper()

    def reset_arm(self):
        """
        reset to rest poses
        """
        for rest_pose, joint_id in zip(self.arm_rest_poses, self.arm_motor_ids):
            p.resetJointState(self.id, joint_id, rest_pose)

    def reset_gripper(self):
        self.open_gripper()

    def open_gripper(self):
        self.move_gripper(self.gripper_range[1])

    def close_gripper(self):
        self.move_gripper(self.gripper_range[0])

    def apply_control(self, action, control_method):
        self.pre_control()
        assert control_method in ('joint', 'end')
        if control_method == 'end':
            x, y, z, roll, pitch, yaw = action
            pos = (x, y, z)
            orn = p.getQuaternionFromEuler((roll, pitch, yaw))
            joint_poses = p.calculateInverseKinematics(self.id, self.eef_id, pos, orn,
                                                       self.arm_lower_limits, self.arm_upper_limits, self.arm_joint_ranges, self.arm_rest_poses,
                                                       maxNumIterations=20)
        elif control_method == 'joint':
            assert len(action) == self.arm_num_dofs
            joint_poses = action
        # arm
        for i, joint_id in enumerate(self.arm_motor_ids):
            p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, joint_poses[i],
                                    force=self.joints[joint_id].maxForce, maxVelocity=self.joints[joint_id].maxVelocity * 0.7, 
                                    positionGain=0.1, velocityGain=0.7)
        self.post_control()

    def move_gripper(self, open_length):
        raise NotImplementedError
    
    def pre_control(self):
        raise NotImplementedError
    
    def post_control(self):
        raise NotImplementedError
    

    def get_joint_obs(self):
        positions = []
        velocities = []
        torques = []
        for joint_id in self.arm_motor_ids:
            pos, vel, _ , torque = p.getJointState(self.id, joint_id)
            velocities.append(vel)
            positions.append(pos)
            torques.append(torque)
        # ee_pos = p.getLinkState(self.id, self.eef_id)[0]
        return dict(positions=positions, velocities=velocities, torques=torques)
    
    def get_rotate_joint_info_all(self):
        id = []
        positions = []
        velocities = []
        torques = []
        for i in range(len(self.rotate_joint_id)):
            pos, vel, _ , torque = p.getJointState(self.id, self.rotate_joint_id[i])
            id.append(self.rotate_joint_id[i])
            positions.append(pos)
            velocities.append(vel)
            torques.append(torque)
        return dict(id=id, positions=positions, velocities=velocities, torques=torques)


