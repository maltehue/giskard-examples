#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch &

# To fix file path 
MUJOCO_WORLD_PATH=${ROS_WS}/src/mujoco_world/mujoco_world
mkdir ${MUJOCO_WORLD_PATH}/mujoco_world
ln -s ${MUJOCO_WORLD_PATH}/model ${MUJOCO_WORLD_PATH}/mujoco_world/model

# Rebuild blockly jupyter extension
cd jupyterlab-blockly-ipylgbst
ln -s ./src/giskard_blocks_and_toolbox.js ../notebooks/blockly.js
jlpm watch &
cd /home/${NB_USER}/giskard_examples

exec "$@"