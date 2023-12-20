#!/usr/bin/env python3
import pybullet as p
from robots.ur5 import UR5
import gin
import time
import pybullet_data
import ros_wrapper.ros_msg 
from ros_wrapper.ros_wrapper import RosWrapper
import os

ROS_CLOCK_TOPIC = "clock"

@gin.configurable
class Environment():
    def __init__(self, setRealTimeSimulation, dt, realtime_factor):
        self.client = p.connect(p.GUI)
        self.robot = UR5()
        p.setRealTimeSimulation(setRealTimeSimulation)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(dt)
        self.dt = dt
        self.time = 0.0
        self.realtime_factor = realtime_factor
        self.ros_wrapper = self.robot.ros_wrapper
        self.ros_wrapper.add_publisher(ROS_CLOCK_TOPIC, ros_wrapper.ros_msg .ROSDtype.CLOCK, False)
        # p.resetDebugVisualizerCamera(1.674, 70, -50.8, [0, 0, 0])+
        self.load_scenes()

    def step(self):
        start_time = time.time()   
        self.time += self.dt
        self.ros_wrapper.ros_time = self.time
        self.ros_wrapper.publish_msg(ROS_CLOCK_TOPIC, ros_wrapper.ros_msg.ROSClock(self.time))

        action = self.robot.set_angle[0]
        self.robot.apply_control(action, "joint")
        p.stepSimulation()
        
        
        
        end_time = time.time()
        elapsed_time = end_time - start_time
        if self.dt / self.realtime_factor - elapsed_time > 0:
            time.sleep(self.dt / self.realtime_factor - elapsed_time)

    def load_scenes(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])

CONFIG_FILE = os.path.dirname(os.path.abspath(__file__)) + "/config/env_default.gin"
gin.parse_config_file(CONFIG_FILE)

if __name__ == "__main__":
    env = Environment()
    while True:
        env.step()