#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

#!/usr/bin/env python3

from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5 as robot
#from pymoveit2.robots import panda as robot
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from shape_msgs.msg import Plane
import yaml
from ament_index_python.packages import get_package_share_directory
import os

def main():
    rclpy.init()
    package_path = get_package_share_directory("multi_robot_arm")
    arm_params = load_yaml(
        package_path, "config/ur/ur5/dual_robot.yaml"
    )

    # Create node for this example
    node1 = Node("ex_pose_goal", namespace=arm_params["arm1"]["name"])
    node2 = Node("ex_pose_goal", namespace=arm_params["arm2"]["name"])

    # Declare parameters for position and orientation
    init_position = [0.2, 0.0, 0.25]
    init_quat_xyzw = [1.0, 0.0, 0.0, 0.0]
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
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node1)
    executor.add_node(node2)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    # position = node1.get_parameter("position").get_parameter_value().double_array_value
    # quat_xyzw = node1.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    # cartesian = node1.get_parameter("cartesian").get_parameter_value().bool_value
    position = -0.0, -0.0, 0.5
    position_1 = 0.2, 0.2, 0.2
    position_2 = 0.2, 0.2, 0.2
    quat_xyzw = 1.0, 0.0, 0.0, 0.0
    # quat_xyzw_2 = 0.0, 0.0, 0.0, 0.0
    cartesian = True

    position_list = list(position)
    position_1_list = list(position_1)
    position_1_list[0] = position_list[0] 
    # - 0.8*float(arm_params["arm1"]["x_pose"])
    position_1_list[1] = position_list[1] 
    # - 0.8*float(arm_params["arm1"]["y_pose"])
    position_1_list[2] = position_list[2]

    position_2_list = list(position_2)
    position_2_list[0] = position_list[0] - 0.8*float(arm_params["arm2"]["x_pose"])
    position_2_list[1] = position_list[1] - 0.8*float(arm_params["arm2"]["y_pose"])
    position_2_list[2] = position_list[2]

    position = tuple(position_list)
    position_1 = tuple(position_1_list)
    position_2 = tuple(position_2_list)

    # Move to pose
    node1.get_logger().info(f"Moving arm to position1: {list(position_1)}")
    node2.get_logger().info(f"Moving arm to position2: {list(position_2)}")

    add_ground_plane_1(node1)
    add_ground_plane_2(node2)
    
    moveit2_1.move_to_pose(position=position_1, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2_2.move_to_pose(position=position_2, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2_1.wait_until_executed()
    moveit2_2.wait_until_executed()
    time.sleep(1)

    moveit2_1.move_to_pose(position=init_position, quat_xyzw=init_quat_xyzw, cartesian=cartesian)
    node1.get_logger().info(f"Moving to initial pose...")

    moveit2_2.move_to_pose(position=init_position, quat_xyzw=init_quat_xyzw, cartesian=cartesian)
    node2.get_logger().info(f"Moving to initial pose...")

    rclpy.shutdown()
    exit(0)

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