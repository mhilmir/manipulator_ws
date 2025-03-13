# sourcing
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source ~/manipulator_ws/install/local_setup.bash

# launch file
ros2 launch open_manipulator_x_moveit_config moveit_core.launch.py