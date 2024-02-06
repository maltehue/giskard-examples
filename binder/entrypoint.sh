#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch &

MUJOCO_WORLD_PATH=${ROS_WS}/src/mujoco_world/mujoco_world
mkdir ${MUJOCO_WORLD_PATH}/mujoco_world
ln -s ${MUJOCO_WORLD_PATH}/model ${MUJOCO_WORLD_PATH}/mujoco_world/model

exec "$@"