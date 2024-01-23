FROM intel4coro/base-notebook:20.04-noetic-full-xpra

RUN pip uninstall -y jlab-enhanced-cell-toolbar
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.2.tar.gz \
  https://raw.githubusercontent.com/yxzhan/extension-examples/main/cell-toolbar/dist/jupyterlab_examples_cell_toolbar-0.1.3.tar.gz
RUN mamba install -y ripgrep jupyterlab-blockly
RUN pip install --upgrade \
  sidecar \
  jupyterlab-git \
  jupyterlab-unfold \
  jupyterlab-search-replace \
  jupyter-archive  \
  jupyterlab-language-pack-de-DE  \
  jupyterlab_execute_time

# Initiate ROS workspace 
ENV ROS_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${ROS_WS}/src
COPY --chown=${NB_USER}:users binder/me ${ROS_WS}/src/me
RUN pip install vcstool
COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN vcs import --input /home/${NB_USER}/noetic.rosinstall --recursive

# Install dependencies
WORKDIR  ${ROS_WS}
USER root
RUN rosdep update && apt update && apt dist-upgrade -y \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN pip install -r src/giskardpy/requirements.txt

# Install missing python packages (should be included in giskardpy/requirements.txt)
RUN pip install termcolor pydot

# Building Custom bullet3 from source with Python 3.10 (in a clumsy way)
WORKDIR /home/${NB_USER}
RUN git clone https://github.com/pybind/pybind11.git \
  && cd pybind11 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
USER root
RUN cd pybind11/build && make install
USER ${NB_USER}

RUN git clone https://github.com/SemRoCo/bullet3.git
COPY --chown=${NB_USER}:users binder/build_cmake_pybullet_3.10_double.sh /home/${NB_USER}/bullet3/
RUN cd bullet3 && ./build_cmake_pybullet_3.10_double.sh
ENV PYTHONPATH=${PYTHONPATH}:/home/${NB_USER}/bullet3/build_cmake/better_python:/home/${NB_USER}/bullet3/examples/pybullet

# Build ROS workspace
WORKDIR  ${ROS_WS}
RUN catkin build

# Update RvizWeb
RUN cd ${ROS_WS}/src/rvizweb && git pull

# Copy the giskard tmp meshes to skip the converting step
COPY --chown=${NB_USER}:users binder/giskard_decomposed_obj ${ROS_WS}/src/giskardpy/tmp
# Copy contents of the repo into the image
COPY --chown=${NB_USER}:users . /home/${NB_USER}/giskard_examples
WORKDIR /home/${NB_USER}/giskard_examples
RUN git config --global --add safe.directory ${PWD}
RUN ln -s ${ROS_WS} ${PWD}/ROS_WS
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/jupyter-settings.json /opt/conda/share/jupyter/lab/settings/overrides.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]

# === Add Stretch robot from giskard:devel ===
# === (To be deleted after stretch add to giskard:master) ===

WORKDIR  ${ROS_WS}/src/giskardpy
ARG GISKARD_COMMIT_PREFIX=https://raw.githubusercontent.com/SemRoCo/giskardpy/ee1daaf2cfdf70c9981d63f6a7dfd5b4626b77ff
RUN cd src/giskardpy/configs/iai_robots && \
  wget ${GISKARD_COMMIT_PREFIX}/src/giskardpy/configs/iai_robots/stretch.py
RUN cd self_collision_matrices/iai && \
  wget ${GISKARD_COMMIT_PREFIX}/self_collision_matrices/iai/stretch.srdf
RUN cd scripts/iai_robots && \
  mkdir stretch && cd stretch && \
  wget ${GISKARD_COMMIT_PREFIX}/scripts/iai_robots/stretch/stretch_standalone.py && \
  chmod +x stretch_standalone.py
WORKDIR /home/${NB_USER}/giskard_examples
# === End ===
