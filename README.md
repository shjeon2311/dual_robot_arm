# Picking and Packing Dual Robot Arm

This GitHub repository contains the code used for the 2024-1 Capstone Design project. The project involved uploading the code from this repository to hardware built using an Arduino Due, Mini Maestro Servo Controller, and Servo Motor. 

You can check out our results at https://www.youtube.com/watch?v=6UXpxrR5yh4&list=LL


## Setup
The package is verified with ROS2 Foxy and Humble.

Dependencies: ROS2 [Foxy or Humble], moveit2, pymoveit2, Gazebo 11



## Clone and Build
```
source /opt/ros/<DISTRO>/setup.bash
mkdir -p robot_ws
cd robot_ws
git clone https://github.com/shjeon2311/dual_robot_arm.git
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Launch

**Console 1: Launch Gazebo**
```
source ./install/setup.bash
ros2 launch multi_robot_arm gazebo_arm.launch.py
```

**Console 2: Run micro-ROS agent for Arm 1 (check the USB Port number)**
```
source ./install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

**Console 3: Run micro-ROS agent for Arm 2 (check the USB Port number)**
```
source ./install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1
```

**Console 4: Integrate Gazebo with Hardware**
```
source ./install/setup.bash
cd ./src/pymoveit2/examples/
python3 ex_servo.py
```




**Console 5: Run task file**
```
source ./install/setup.bash
cd ./src/pymoveit2/examples/
python3 task_box.py
(or python3 task_bag.py)
```



To check the subscribtion of the topic for the motor angle values, you can follow the additional steps:

**(Console 6: Run task file)**
```
source ./install/setup.bash
ros2 topic echo /micro_ros_arduino_subscriber_1
```

**(Console 7: Run task file)**
```
source ./install/setup.bash
ros2 topic echo /micro_ros_arduino_subscriber_2
```
