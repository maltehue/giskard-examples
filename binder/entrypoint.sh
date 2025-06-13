#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait dlr_kitchen upload_dlr_kitchen.launch &
roslaunch --wait giskardpy_ros giskardpy_pr2_standalone_vrb.launch &
roslaunch --wait rvizweb rvizweb.launch config_file:=launch/rvizweb_config/hsr_mujoco.json &


jupyter lab workspaces import binder/vis-with-terminal.jupyterlab-workspace

exec "$@"
