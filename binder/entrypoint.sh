#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=launch/rvizweb_config/hsr_mujoco.json &
roslaunch --wait dlr_kitchen upload_dlr_kitchen.launch &
roslaunch --wait giskardpy_ros giskardpy_pr2_standalone_vrb.launch &
rviz -d /home/jovyan/giskard_examples/launch/rvizweb_config/bmp.rviz &

jupyter lab workspaces import binder/vis-with-terminal.jupyterlab-workspace

exec "$@"
