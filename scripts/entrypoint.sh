#!/bin/bash
set -e
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -e build ]
then
echo "Build dir exist"
else
colcon build
fi
source "install/setup.bash"
ros2 launch kinco_driver kinco_driver.launch.py
while true; do sleep 1000; done
exec "$@"
