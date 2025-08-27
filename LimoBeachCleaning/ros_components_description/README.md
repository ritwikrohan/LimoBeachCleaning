# ros_components_description

URDF models of sensors and other components offered alongside with Husarion robots

## Including sensor

First build the package by running:

```bash
# create workspace folder and clone ros_components_description
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/husarion/ros_components_description.git src/ros_components_description

# in case the package will be used within simulation
export HUSARION_ROS_BUILD_TYPE=simulation

rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

To include the sensor, use the following code:

```xml
<!-- include file with definition of xacro macro of sensor -->
<xacro:include filename="$(find ros_components_description)/urdf/slamtec_rplidar_s1.urdf.xacro" ns="lidar" />

<!-- evaluate the macro and place the sensor on robot -->
<xacro:lidar.slamtec_rplidar_s1
  parent_link="cover_link"
  xyz="0.0 0.0 0.0"
  rpy="0.0 0.0 0.0" />
```

A list of parameters can be found here:

- `parent_link` [*string*, default: **None**] parent link to which sensor should be attached.
- `xyz` [*float list*, default: **None**] 3 float values defining translation between base of a sensor and parent link. Values in **m**.
- `rpy` [*float list*, default: **None**] 3 float values define rotation between parent link and base of a sensor. Values in **rad**.
- `namespace` [*string*, default: **None**] global namespace common to the entire robot.
- `device_namespace` [*string*, default: **None**] local namespace allowing to distinguish two identical devices from each other.

- `model` [*string*, default: **None**] model argument that appears when you want to load the appropriate model from a given manufacturer.

Some sensors can define their specific parameters. Refer to their definition for more info.
