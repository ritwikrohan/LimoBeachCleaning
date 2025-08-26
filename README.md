# Limo Beach Cleaning Robot

Autonomous beach cleaning system using AgileX Limo Cobot with ROS2 for environmental restoration tasks.

## Overview

This project implements an autonomous beach cleaning robot that identifies and collects trash using computer vision and robotic manipulation. The system combines SLAM navigation, object detection, person following, and precision grasping to operate in unstructured outdoor environments.

## Demo

### Trash Detection and Pickup
[![Trash Pickup Demo](https://img.youtube.com/vi/m8PfNT-SJBM/0.jpg)](https://www.youtube.com/watch?v=m8PfNT-SJBM)
*Click to watch: Limo robot detecting and collecting trash in Gazebo simulation*

### Person Following with YOLO
[![Person Following Demo](https://img.youtube.com/vi/0Gs47wrVdOE/0.jpg)](https://www.youtube.com/watch?v=0Gs47wrVdOE)
*Click to watch: YOLO-based person detection and PID-controlled following*

## Key Features

- **Autonomous Navigation**: RTAB-Map visual-LiDAR fusion for robust outdoor SLAM
- **Trash Detection**: Real-time object detection using RANSAC plane fitting and Euclidean clustering
- **Person Following**: YOLOv8 person detection with PID-based following at safe distance
- **Precision Manipulation**: MoveIt2-based arm control for pick-and-place operations
- **Multi-Robot Support**: Compatible with UR3e/UR5e arms, Husarion ROSbot XL, and other platforms
- **Multi-terrain Capable**: Operates on sand, grass, and paved surfaces
- **Simulation-First Development**: Full Gazebo Ignition environment for safe testing

## Performance Metrics

| Metric | Value | Conditions |
|--------|-------|------------|
| Grasp Success Rate | 85% | 100+ trials on varied objects |
| Detection Accuracy | 92% | Mixed lighting conditions |
| Person Following Distance | 1.5m ± 0.2m | PID-controlled tracking |
| Navigation Accuracy | ±15cm | 500m outdoor trajectories |
| Cycle Time | 45 seconds | Detection to disposal |

## Technical Stack

- **Platform**: AgileX Limo Cobot (4WD + 6DOF Arm)
- **Framework**: ROS2 Humble
- **Simulation**: Gazebo Ignition Fortress
- **SLAM**: RTAB-Map (RGB-D + LiDAR fusion)
- **Object Detection**: YOLOv8 for trash and person detection
- **Manipulation**: MoveIt2 + ros2_control
- **Perception**: PCL, OpenCV, RANSAC + Euclidean clustering
- **Control**: PID controllers for person following
- **Localization**: EKF sensor fusion (IMU + Odometry + Vision)

## Installation

### Prerequisites
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop-full

# MoveIt2 and Navigation
sudo apt install ros-humble-moveit ros-humble-navigation2

# RTAB-Map
sudo apt install ros-humble-rtabmap-ros

# Gazebo Ignition
sudo apt install ignition-fortress
```

### Build from Source
```bash
# Clone repository
git clone https://github.com/ritwikrohan/LimoBeachCleaning.git
cd LimoBeachCleaning

# Download large model files (stored separately due to size)
./download_assets.sh  # Downloads beach world models from Google Drive

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Simulation Environment
```bash
# Start Gazebo with beach world
ros2 launch beach_world beach_simulation.launch.py

# Launch robot with all sensors
ros2 launch limo_robot robot_spawn.launch.py
```

### Run Autonomous Cleaning
```bash
# Start SLAM and navigation
ros2 launch limo_rtabmap rtabmap.launch.py
ros2 launch nav2_application navigation.launch.py

# Launch perception pipeline
ros2 launch advanced_perception trash_detection.launch.py

# Start YOLO detection for trash and person
ros2 launch yolo yolo_detection.launch.py

# Enable person following mode
ros2 run nav person_follower_pid.py

# Start manipulation controller
ros2 launch limo_moveit_config moveit_planning_execution.launch.py

# Begin autonomous cleaning behavior
ros2 run nav2_application beach_cleaning_behavior.py
```

### Alternative Robot Platforms
```bash
# Launch with Universal Robot UR3e/UR5e
ros2 launch universal_robot_ros2 ur_control.launch.py ur_type:=ur3e

# Launch with Husarion ROSbot XL
ros2 launch rosbot_xl_ros simulation.launch.py

# Launch with custom cobot configuration
ros2 launch cobot cobot_bringup.launch.py
```

## Repository Structure

```
LimoBeachCleaning/
├── advanced_perception/      # Perception algorithms
│   ├── src/
│   │   ├── trash_detector.cpp      # RANSAC + clustering
│   │   └── clustering_node.py
│   └── config/
├── yolo/                    # YOLO detection
│   ├── src/
│   │   ├── yolo_detector.py       # YOLOv8 implementation
│   │   └── person_tracker.py      # Person detection & tracking
│   └── weights/
├── nav/                     # Navigation & person following
│   ├── src/
│   │   └── person_follower_pid.py  # PID controller for following
│   └── config/
├── beach_world/            # Simulation environment
│   ├── worlds/
│   ├── models/            # [Large files - download separately]
│   └── launch/
├── limo_robot/            # Limo robot description and control
├── universal_robot_ros2/  # UR3e/UR5e support
├── rosbot_xl_ros/        # Husarion ROSbot XL support
├── cobot/                # Additional cobot configurations
├── limo_moveit_config/   # MoveIt2 configuration
├── limo_rtabmap/        # SLAM configuration
└── nav2_application/    # Navigation behaviors
```

## Technical Implementation

### YOLO Person Following Pipeline
1. **Person Detection**: YOLOv8 runs at 30 FPS to detect person bounding boxes
2. **Distance Estimation**: Depth camera provides person distance
3. **PID Control**: Maintains 1.5m following distance with angular and linear velocity control
4. **Safety Features**: Emergency stop if person moves too close (<0.5m)

### Perception Pipeline
1. **LiDAR Processing**: Voxel grid downsampling → Statistical outlier removal
2. **Ground Segmentation**: RANSAC plane fitting with adaptive thresholds
3. **Object Clustering**: Euclidean clustering with size-based filtering
4. **Trash Classification**: YOLOv8 + geometric heuristics for graspability

### Navigation Strategy
- **Global Planning**: A* over 2D costmap from RTAB-Map
- **Local Planning**: DWB controller with custom critics for rough terrain
- **Recovery Behaviors**: Rotation recovery, backup, and clearing costmaps
- **Person Following Mode**: Overrides navigation goals when person detected

### Multi-Robot Compatibility
- **Universal Robot Integration**: Support for UR3e/UR5e arms via universal_robot_ros2 package
- **Husarion Platform**: ROSbot XL can be used as alternative mobile base
- **Modular Design**: Easy switching between robot platforms through launch parameters

## Contact

**Ritwik Rohan**  
Robotics Engineer | Johns Hopkins MSE '25  
Email: ritwikrohan7@gmail.com  
LinkedIn: [linkedin.com/in/ritwik-rohan](https://linkedin.com/in/ritwik-rohan)

---