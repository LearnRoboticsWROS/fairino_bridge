# 🦾 FAIRINO + MoveIt 2 Integration (ROS 2 Humble)

This guide explains how to **connect MoveIt 2** to a real **FAIRINO FR3 robot** using the `fairino_bridge` package as a communication layer.  
It assumes the official `frcobot_ros2` stack is already installed and focuses on **modifying the MoveIt configuration package** (`fairino3mt_v6_moveit2_config`) so that MoveIt can plan and execute trajectories on the *real robot* (not the fake ros2_control simulation).

---

## 🧩 Architecture Overview

The system architecture separates **low-level hardware** (FAIRINO controller) from **high-level motion planning** (MoveIt).  
The `fairino_bridge` acts as the translator between both systems.


  +-------------------+
  |     MoveIt 2      |
  | (move_group node) |
  +-------------------+
            |
            | FollowJointTrajectory Action
            v
  +--------------------------+
  |  fairino_bridge package  |
  | - joint_state_bridge     | --> /joint_states
  | - follow_joint_trajectory_server
  +--------------------------+
            |
            | FAIRINO service API
            v
  +--------------------------+
  |  fairino_hardware node   |
  | (ros2_cmd_server)        |
  +--------------------------+


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
´´´

## 1️⃣ Create the Package

In your workspace:

clone this repository