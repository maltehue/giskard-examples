#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=launch/rvizweb_config/pr2.json &
roslaunch --wait giskardpy giskardpy_pr2_standalone_vrb.launch &
roslaunch --wait iai_maps upload_kitchen_obj.launch &

exec "$@"