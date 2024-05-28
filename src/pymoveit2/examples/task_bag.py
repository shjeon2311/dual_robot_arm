from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5 as robot
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from shape_msgs.msg import Plane
from std_msgs.msg import Float32MultiArray
import math
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import concurrent.futures
import time

# Create publisher
class MyPublisher1(Node):
    def __init__(self):
        super().__init__('my_publisher', namespace='arm1')
        self.publisher_ = self.create_publisher(
            Float32MultiArray,  
            '/my_topic1',       
            10)  
    def publish_data(self, data):
        msg = Float32MultiArray()  
        msg.data = data 
        self.publisher_.publish(msg)  

class MyPublisher2(Node):
    def __init__(self):
        super().__init__('my_publisher', namespace='arm2')
        self.publisher_ = self.create_publisher(
            Float32MultiArray,  
            '/my_topic2',       
            10)  
    def publish_data(self, data):
        msg = Float32MultiArray()  
        msg.data = data 
        self.publisher_.publish(msg) 

def main():
    rclpy.init()
    nodea = MyPublisher1()
    nodeb = MyPublisher2()

    username = os.getenv("USERNAME") or os.getenv("USER")
    package_path = get_package_share_directory("multi_robot_arm")
    config_file_path = f"/home/{username}/robot_ws/src/multi_robot_arm/config/ur/ur5/dual_robot.yaml"
    arm_params = load_yaml(package_path, config_file_path)

    # Create node for this example
    node1 = Node("ex_pose_goal", namespace=arm_params["arm1"]["name"])
    node2 = Node("ex_pose_goal", namespace=arm_params["arm2"]["name"])

    # Declare parameters for position and orientation
    init_position = [0.15, 0.0, 0.25] 
    init_quat_xyzw = [0.0, 0.0, 1.0, 0.0]
    node1.declare_parameter("position", init_position)
    node1.declare_parameter("quat_xyzw", init_quat_xyzw)
    node1.declare_parameter("cartesian", True)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2_1 = MoveIt2(
        node=node1,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group
    )

    moveit2_2 = MoveIt2(
        node=node2,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group
    )
    
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(10)
    executor.add_node(node1)
    executor.add_node(node2)
    executor.add_node(nodea)
    executor.add_node(nodeb)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    publish_open = [130.]
    publish_box = [80.]
    publish_close = [40.]

    pos_1 = [[0.19, 0.08, 0.30], [0.19, 0.15, 0.19], [0.18, 0.0, 0.28], [0.22, -0.14, 0.22]]
    pos_2 = [[0.18, -0.15, 0.22], [0.21, -0.15, 0.06], [0.2, -0.1, 0.22], [0.21, 0.01, 0.28]]
    quat_1 = [[0.1, 0.1, -0.7, 0.7], [0.5, 0.5, 0.5, 0.5]]
    quat_2 = [[-0.1, -0.1, -0.7, 0.7], [-0.5, -0.5, 0.5, 0.5]]
    cartesian = True

    add_ground_plane_1(node1)
    add_ground_plane_2(node2)
    sleep_time = 3

    # Move to initial pose
    moveit2_1.move_to_pose(position=init_position, quat_xyzw=quat_1[0], cartesian=cartesian)
    moveit2_1.wait_until_executed()
    moveit2_2.move_to_pose(position=init_position, quat_xyzw=quat_2[0], cartesian=cartesian)
    moveit2_2.wait_until_executed()
    time.sleep(2)
    open_gripper(nodea, publish_open)
    time.sleep(4)
    open_gripper(nodeb, publish_open)
    time.sleep(4)
    
    # 1. Bring the shopping bag
    moveit2_1.move_to_pose(position=pos_1[0], quat_xyzw=quat_1[0], cartesian=cartesian)
    moveit2_1.wait_until_executed()
    moveit2_1.move_to_pose(position=pos_1[1], quat_xyzw=quat_1[0], cartesian=cartesian)
    moveit2_1.wait_until_executed()
    time.sleep(3)
    close_gripper(nodea, publish_close)
    time.sleep(4)
    open_gripper(nodeb, publish_open)
    time.sleep(4)
    moveit2_1.move_to_pose(position=pos_1[2], quat_xyzw=quat_1[0], cartesian=cartesian)
    
    # 2. Bring the item
    moveit2_2.move_to_pose(position=pos_2[0], quat_xyzw=quat_2[1], cartesian=cartesian)
    time.sleep(5)
    moveit2_2.move_to_pose(position=pos_2[1], quat_xyzw=quat_2[1], cartesian=cartesian)
    moveit2_2.wait_until_executed()
    time.sleep(4)
    close_gripper(nodeb, publish_close)
    time.sleep(4)
    
    # 3. Place the item in the shopping bag
    moveit2_2.move_to_pose(position=pos_2[2], quat_xyzw=quat_2[0], cartesian=cartesian)
    moveit2_2.wait_until_executed()
    moveit2_2.move_to_pose(position=pos_2[3], quat_xyzw=quat_2[0], cartesian=cartesian)
    moveit2_2.wait_until_executed()
    time.sleep(3)
    open_gripper(nodeb, publish_open)
    time.sleep(5)
    
    # 4. Move the shopping bag aside
    moveit2_1.move_to_pose(position=pos_1[3], quat_xyzw=quat_1[0], cartesian=cartesian)
    moveit2_2.move_to_pose(position=init_position, quat_xyzw=quat_2[0], cartesian=cartesian)
    time.sleep(4)
    open_gripper(nodea, publish_open)
    time.sleep(4)

    moveit2_1.move_to_pose(position=init_position, quat_xyzw=quat_1[0], cartesian=cartesian)
    moveit2_1.wait_until_executed()

    node1.destroy_node()
    node2.destroy_node()
    nodea.destroy_node()
    nodeb.destroy_node()
    rclpy.shutdown()
    exit(0)

def open_gripper(node, data_to_publish):
    node.publish_data(data_to_publish)

def close_gripper(node, data_to_publish):
    node.publish_data(data_to_publish)

def add_ground_plane_1(node1):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True
    
    publisher_ = node1.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

def add_ground_plane_2(node2):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True
    
    publisher_ = node2.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

def load_yaml(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

if __name__ == "__main__":
    main()