#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=launch/rvizweb_config/hsr_mujoco.json &
roslaunch --wait launch/display_lab_env.launch &
roslaunch --wait giskardpy giskardpy_kevin_standalone.launch &

jupyter lab workspaces import binder/vis-with-terminal.jupyterlab-workspace

exec "$@"