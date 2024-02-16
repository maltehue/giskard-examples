# Giskard examples

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/yxzhan/giskard-examples.git/mujoco_actions_devel)

[Giskard](https://github.com/SemRoCo/giskardpy) examples in Jupyter notebooks

## Quick Start

### Option 1: Test Image Locally (Under repo directory)

- Run Docker image with X-forwarding

  ```bash
  xhost +local:docker && \
  docker compose -f ./binder/docker-compose.yml up  --build && \
  xhost -local:docker
  ```

- Open Web browser and go to http://localhost:8888/lab/tree/giskard_examples/notebooks/playground.ipynb

- To stop and remove container:

  ```bash
  docker compose -f ./binder/docker-compose.yml down
  ```

### Option 2: Run on BinderHub

https://binder.intel4coro.de/v2/gh/yxzhan/giskard-examples.git/mujoco_actions_devel?urlpath=%2Flab%2Ftree%2Fnotebooks%2Fplayground.ipynb

## Tutorials of giskardpy

https://github.com/SemRoCo/giskardpy/wiki

## Publications

[S. Stelter, G. Bartels and M. Beetz, “An open-source motion planning framework for mobile manipulators using constraint-based task space control with linear MPC,” 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Kyoto, Japan, 2022, pp. 1671-1678, doi: 10.1109/IROS47612.2022.9982245.
](https://ieeexplore.ieee.org/document/9982245)