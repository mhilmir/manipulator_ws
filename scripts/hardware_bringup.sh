# sourcing
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source ~/Manipulator_ws/install/local_setup.bash

# grant access
sudo chmod 777 /dev/ttyACM0

# launch file bringup
ros2 launch open_manipulator_x_bringup hardware.launch.py port_name:=/dev/ttyACM0