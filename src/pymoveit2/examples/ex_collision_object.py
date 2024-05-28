from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5 as robot1, panda as robot2
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Plane

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_object")

    # Declare parameters for position and orientation for each arm
    node.declare_parameter("position_arm1", [0.25, 0.0, 1.0])
    node.declare_parameter("quat_xyzw_arm1", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("position_arm2", [-0.25, 0.0, 1.0])  # Different position for arm2
    node.declare_parameter("quat_xyzw_arm2", [0.0, 0.0, 0.0, 1.0])  # Different orientation for arm2
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interfaces for each robot arm
    moveit2_arm1 = MoveIt2(
        node=node,
        joint_names=robot1.joint_names(),
        base_link_name=robot1.base_link_name(),
        end_effector_name=robot1.end_effector_name(),
        group_name=robot1.MOVE_GROUP_ARM,
        callback_group=callback_group
    )
    moveit2_arm2 = MoveIt2(
        node=node,
        joint_names=robot2.joint_names(),
        base_link_name=robot2.base_link_name(),
        end_effector_name=robot2.end_effector_name(),
        group_name=robot2.MOVE_GROUP_ARM,
        callback_group=callback_group
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)  # Third thread for both arms
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters for each arm
    position_arm1 = node.get_parameter("position_arm1").get_parameter_value().double_array_value
    quat_xyzw_arm1 = node.get_parameter("quat_xyzw_arm1").get_parameter_value().double_array_value
    position_arm2 = node.get_parameter("position_arm2").get_parameter_value().double_array_value
    quat_xyzw_arm2 = node.get_parameter("quat_xyzw_arm2").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move arm1 to its pose
    node.get_logger().info(
        f"Moving arm 1 to pose: {{position: {list(position_arm1)}, quat_xyzw: {list(quat_xyzw_arm1)}}}"
    )
    try:
        add_ground_plane(node)
        moveit2_arm1.move_to_pose(position=position_arm1, quat_xyzw=quat_xyzw_arm1, cartesian=cartesian)
        moveit2_arm1.wait_until_executed()
    except Exception as err:
        node.get_logger().info(f'Exception occured for arm 1. {err}')

    # Move arm2 to its pose
    node.get_logger().info(
        f"Moving arm 2 to pose: {{position: {list(position_arm2)}, quat_xyzw: {list(quat_xyzw_arm2)}}}"
    )
    try:
        add_ground_plane(node)
        moveit2_arm2.move_to_pose(position=position_arm2, quat_xyzw=quat_xyzw_arm2, cartesian=cartesian)
        moveit2_arm2.wait_until_executed()
    except Exception as err:
        node.get_logger().info(f'Exception occured for arm 2. {err}')

    node.get_logger().info(f'Movement completed for both arms')
    rclpy.shutdown()
    exit(0)


# 나머지 함수들은 동일하게 유지됩니다.
def get_planning_scene(node):

    # Create a client for the GetPlanningScene service
    get_scene_service = node.create_client(GetPlanningScene, "get_planning_scene")

    # Wait for the service to be available
    while not get_scene_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the get_planning_scene service...")

    # Create a request object
    request = GetPlanningScene.Request()

    # Set the desired scene components to be returned
    request.components.components = request.components.WORLD_OBJECT_NAMES
    # Call the GetPlanningScene service
    future = get_scene_service.call(request)

    if future is not None:
        if future.scene is not None:
            scene = future.scene
            for obj in scene.world.collision_objects:
                print("Collision Object ID:", obj.id)
                print("Collision Object Type:", obj.type)
                print("Collision Object Plane:", obj.planes)
                print("Collision Object Plane Pose:", obj.plane_poses)
                print("-----------------------------")
        return future.scene

    return None

def add_ground_plane(node):

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
    
    publisher_ = node.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

if __name__ == "__main__":
    main()
