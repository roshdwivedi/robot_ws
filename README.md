# My Bot Description
This project provides a ROS 2 package, `my_bot_description`, for simulating a differential drive robot in Gazebo and visualizing it in RViz.

---

## Features

- **URDF/Xacro-based Robot Description**
- **Gazebo Simulation with Plugins**
- **RViz Visualization**
- **SLAM Toolbox for Mapping**
- **ROS-Gazebo Topic Bridging**

---

## Installation

1. Clone the repository:
    ```bash
    cd ~/robot_ws/src
    git clone <repository_url> my_bot_description
    ```
2. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build and source the workspace:
    ```bash
    cd ~/robot_ws
    colcon build
    source install/setup.bash
    ```

---

## Usage

Launch the simulation:
```bash
ros2 launch my_bot_description sim.launch.py
```

Control the robot:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}"
```

---
