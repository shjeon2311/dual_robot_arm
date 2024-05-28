import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import math
import numpy as np
import os
from std_msgs.msg import Float32MultiArray
import time

joint_positions1 = []
joint_positions2 = []
received_data1 = [0.0]
received_data2 = [0.0]

class JointTrajectoryControllerStateSubscriber1(Node):

    def __init__(self):
        super().__init__('joint_trajectory_controller_state_subscriber1')
        self.subscription1 = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm1/arm_controller/state',
            self.joint_trajectory_state_callback1,
            10)
        self.received_data1 = []
        self.subscription1 = self.create_subscription(
            Float32MultiArray,  
            '/my_topic1',       
            self.listener_callback1,  
            10) 
        
    def joint_trajectory_state_callback1(self, msg):
        global joint_positions1
        joint_positions1 = [math.degrees(position) for position in msg.actual.positions]
        joint_positions1 = np.round(joint_positions1, 1)

    def listener_callback1(self, msg):
        global received_data1
        received_data1 = msg.data

class JointTrajectoryControllerStateSubscriber2(Node):

    def __init__(self):
        super().__init__('joint_trajectory_controller_state_subscriber2')
        self.subscription2 = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm2/arm_controller/state',
            self.joint_trajectory_state_callback2,
            10)
        self.received_data2 = []
        self.subscription2 = self.create_subscription(
            Float32MultiArray,  
            '/my_topic2',      
            self.listener_callback2,  
            10) 
        
    def joint_trajectory_state_callback2(self, msg):
        global joint_positions2
        joint_positions2 = [math.degrees(position) for position in msg.actual.positions]
        joint_positions2 = np.round(joint_positions2, 1)

    def listener_callback2(self, msg):
        global received_data2
        received_data2 = msg.data


def main(args=None):
    rclpy.init(args=args)
    node1 = JointTrajectoryControllerStateSubscriber1()
    node2 = JointTrajectoryControllerStateSubscriber2()
    while(1): 
        global received_data1 
        rclpy.spin_once(node1)
        joint_positions1[5] = received_data1[0]
        rclpy.spin_once(node2)
        joint_positions2[5] = received_data2[0]
        formatted_positions1 = '[' + ', '.join(map(str, joint_positions1)) + ']'
        formatted_positions2 = '[' + ', '.join(map(str, joint_positions2)) + ']'
        command = ['ros2 topic pub /micro_ros_arduino_subscriber_1 --once  std_msgs/msg/Float32MultiArray', 
                   '"{data: ' + formatted_positions1 + '}" & ros2 topic pub /micro_ros_arduino_subscriber_2 --once  std_msgs/msg/Float32MultiArray', 
                   '"{data: ' + formatted_positions2 + '}"']  
        command_str = " ".join(command)
        os.system(command_str)
    node1.destroy_node()
    node2.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()