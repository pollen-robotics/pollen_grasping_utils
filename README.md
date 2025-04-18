# pollen_grasping_utils

A collection of utility functions and launch helpers for working with grasp poses in ROS 2, specifically tailored for use with Reachy and other Pollen Robotics applications.

## Features

### 🧠 Grasp Pose Utilities
- Generate `GraspPose` messages from:
  - Euler angles
  - Quaternions
  - Homogeneous transformation matrices
- Create corresponding `Pose` messages using the same formats

### 🎯 Visualization Tools
- Build RViz markers for visualizing grasp poses as fork-like grippers
- Customize color, size, orientation, and lifetime of markers

### ⚙️ Config Management
- Load grasping configuration files (`.yaml`) from the `config/` directory
- Integrate with ROS 2 launch files via `DeclareLaunchArgument`
- Pretty-print loaded configurations for clarity during debug/logging

## Usage

Import and use in your own ROS 2 nodes:

```python
from pollen_grasping_utils import get_grasp_marker, get_pose_msg_from_euler, get_grasp_pose_msg_from_quaternion
```

Launch integration:

```python
from pollen_grasping_utils.config import get_grasping_config_launch_argument, load_grasping_config
```

## Configuration

Place your `.yaml` grasp configuration files in `pollen_grasping_utils/config/`.

At launch, use the `grasping_config` argument to select which config file to load:

```bash
ros2 launch your_package your_launch_file.launch.py grasping_config:=custom_grasp_config
```
