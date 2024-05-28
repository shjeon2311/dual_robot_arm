# Picking and Packing Dual Robot Arm




# Setup
The package is verified with ROS2 Foxy and Humble.

Dependencies: ROS2 [Foxy or Humble], moveit2, pymoveit2, Gazebo 11

# Clone and Build
```
source /opt/ros/<DISTRO>/setup.bash
mkdir -p robot_ws
cd robot_ws
git clone https://github.com/shjeon2311/dual_robot_arm.git
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

# Launch
Console A: Launch Gazebo
```
source ./install/setup.bash
ros2 launch multi_robot_arm gazebo_arm.launch.py
```
Console B: 
source ./install/setup.bash
cd ./src/pymoveit2/examples
python ex_pose_goal.py --ros-args -r __ns:=/arm1 -p position:=[0.5, 0.4,0.2]
