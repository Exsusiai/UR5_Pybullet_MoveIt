import rospy
import numpy as np
from ros_wrapper.ros_msg import ROSDtype, RobotJointState, ImuData, data_to_ros_msg
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
import time

class RosWrapper:
    def __init__(self, rosnode_name): #if rosnode_name is empty, ros_init will not be called, since only one node for each script
        self.publishers = {} # name: [dtype, publisher]
        self.subscribers = {} # name: dtype, data
        self.rosnode_name = rosnode_name
        self.use_sim_time = True
        self.ros_time = 0
        if rosnode_name:
            rospy.init_node(rosnode_name)
            rospy.loginfo("ROS wrapper init, node name = " + rosnode_name + ".")

    def add_publisher(self, topic, dtype, use_namespace=True, queue=5):
        full_topic = topic
        if use_namespace:
            full_topic = self.rosnode_name + '/' + topic
        pub = rospy.Publisher(full_topic, dtype.value, queue_size=queue)
        self.publishers[topic] = [dtype, pub]

    def add_subscriber(self, topic, dtype, data_handle, use_namespace=True):
        """
        subsribe to a specific topic to update value for data_handle.
        @param data_handle: NOTE: must be mutable objects, including list, dict, and set, bytearray
        """
        assert type(data_handle) in (list, dict, set, bytearray), "data_handle must be mutable objects, including list, dict, and set, bytearray."
        full_topic = topic
        if use_namespace:
            full_topic = self.rosnode_name + '/' + topic
        full_topic = "/" + full_topic
        rospy.Subscriber(full_topic, dtype.value, self.topic_callback)
        self.subscribers[full_topic] = [dtype, data_handle]
        
    def topic_callback(self, msg):
        topic = msg._connection_header['topic']
        if self.subscribers[topic][0] == ROSDtype.FLOAT:
            self.subscribers[topic][1][0] = type(self.subscribers[topic][1][0])(msg.data)
        elif self.subscribers[topic][0] == ROSDtype.FLOAT_ARRAY:
            assert len(self.subscribers[topic][1][0]) == len(list(msg.data)), "inbound float array leng"
            self.subscribers[topic][1][0] = list(msg.data)

    def publish_msg(self, topic, msg):
        assert topic in self.publishers, "topic not registered!"
        self.publishers[topic][1].publish(data_to_ros_msg(msg, self.publishers[topic][0], self.ros_time))

if __name__ == "__main__":
    wrapper = RosWrapper("test_node")
    test_data = [[1,2]]
    wrapper.add_subscriber("test", ROSDtype.FLOAT_ARRAY, test_data)
    while True:
        time.sleep(0.1)
        print(test_data[0])
        
    