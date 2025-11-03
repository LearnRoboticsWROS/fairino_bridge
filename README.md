# 🦾 FAIRINO + MoveIt 2 Integration (ROS 2 Humble)

This guide explains how to **connect MoveIt 2** to a real **FAIRINO FR3 robot** using the `fairino_bridge` package as a communication layer.  
It assumes the official `frcobot_ros2` stack is already installed and focuses on **modifying the MoveIt configuration package** (`fairino3mt_v6_moveit2_config`) so that MoveIt can plan and execute trajectories on the *real robot* (not the fake ros2_control simulation).

---

## 🧩 Architecture Overview

The system architecture separates **low-level hardware** (FAIRINO controller) from **high-level motion planning** (MoveIt).  
The `fairino_bridge` acts as the translator between both systems.


**Key concept:**  
- MoveIt sends trajectories → `fairino_bridge` converts them into FAIRINO controller commands.  
- FAIRINO publishes joint positions → `joint_state_bridge` converts them to `/joint_states`.  
- MoveIt uses `/joint_states` to track the *real robot pose*.

---

## ⚙️ Prerequisites

- Ubuntu 22.04  
- ROS 2 Humble (`ros-humble-desktop`)  
- FAIRINO `frcobot_ros2` stack  
- `fairino_bridge` package already built inside the same workspace (`~/fr_ws`)

Test your FAIRINO interface before proceeding:

```bash
ros2 run fairino_hardware ros2_cmd_server
ros2 topic echo /nonrt_state_data 
```

- inside the src folder of ~/fr_ws, clone this repository fairino_bridge
- make sure that it has the following structure:
fairino_bridge/
 ├── package.xml
 ├── CMakeLists.txt
 ├── launch/
 │    └── bridge.launch.py
 └── src/
      ├── joint_state_bridge.cpp
      └── follow_joint_trajectory_server.cpp

## ⚙️ Modifications to the frcobot_ros2 package
- Modify MoveIt Configuration (fairino3mt_v6_moveit2_config)
- open config/fairino3mt_v6_robot.urdf.xacro and Comment out the ros2_control part:

```bash
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="fairino3mt_v6_robot">
  <xacro:arg name="initial_positions_file" default="initial_positions.yaml" />

  <!-- Import URDF -->
  <xacro:include filename="$(find fairino_description)/urdf/fairino3_mt_v6.urdf" />

  <!-- Disable ros2_control -->
  <!-- <xacro:include filename="fairino3mt_v6_robot.ros2_control.xacro" />
  <xacro:fairino3mt_v6_robot_ros2_control name="FakeSystem" initial_positions_file="$(arg initial_positions_file)"/> -->
</robot>
```

- open the file /config/moveit_controllers.yaml file and update the name of the controller with the one setup in the follow_joint_trajectory_server.xpp

```bash
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - moveit_joint_controller

  moveit_joint_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
```
- inside the /config folder of the moveit config package add move_group.yaml file
```bash
move_group:
  ros__parameters:
    use_sim_time: false
    publish_robot_description_semantic: true
    publish_robot_description: true
    allow_trajectory_execution: true
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
    controllers_file: config/moveit_controllers.yaml
```

- open the file ros2_controllers.yaml and comment out the fairino controller:
```bash
# This config file is used by ros2_control
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # fairino3mt_controller:
    #   type: joint_trajectory_controller/JointTrajectoryController


    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# fairino3mt_controller:
#   ros__parameters:
#     joints:
#       - joint1
#       - joint2
#       - joint3
#       - joint4
#       - joint5
#       - joint6
#     command_interfaces:
#       - position
#     state_interfaces:
#       - position
```

- Modify the file demo.launch.py:
```bash
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("fairino3mt_v6_robot", package_name="fairino3mt_v6_moveit2_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('fairino3mt_v6_moveit2_config')

    # Costruisci la configurazione MoveIt (URDF, SRDF, Kinematics, Controllers)
    moveit_config = (
        MoveItConfigsBuilder("fairino3mt_v6_robot", package_name="fairino3mt_v6_moveit2_config")
        .robot_description(file_path="config/fairino3mt_v6_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/fairino3mt_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Robot state publisher (serve a MoveIt per leggere la posizione attuale)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Nodo move_group principale
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"controllers_file": os.path.join(pkg_share, "config", "moveit_controllers.yaml")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz configurato
    rviz_config_path = os.path.join(pkg_share, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[moveit_config.to_dict()],
    )

    # Spawner dei controller gestiti da ROS2 Control (opzionale se usi il bridge)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    moveit_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["moveit_joint_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delay_moveit_joint_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[moveit_joint_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_moveit_joint_controller,
    ])
```

## Build and run

- each terminal that you about to open, make sure to source the environment
```bash
cd ~/fr_ws
colcon build
source install/setup.bash
```

- Terminal 1:
```bash
ros2 run fairino_hardware ros2_cmd_server
```

- Terminal 2:
```bash
ros2 launch fairino_bridge bridge.launch.py
```

- Terminal 3:
```bash
ros2 launch fairino3mt_v6_moveit2_config demo.launch.py
```


