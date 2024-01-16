#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch interactive_markers:=True &

exec "$@"