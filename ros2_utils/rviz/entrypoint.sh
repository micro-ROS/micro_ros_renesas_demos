#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/renesas/install/setup.bash"

exec ros2 launch micro-ros_motor_demo launch_motor_demo.launch.py