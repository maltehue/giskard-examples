FROM intel4coro/base-notebook:20.04-noetic-vnc

# update jupyter extensions
USER ${NB_USER}
RUN pip install --upgrade \
  jupyterlab~=4.0.0 \
  sidecar \
  jupyterlab-git \
  jupyterlab-unfold \
  jupyterlab_execute_time \
  jupyter-collaboration

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

# Building Custom bullet3 from source with Python 3.10
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

# Build qpSWIFT
RUN git clone https://github.com/qpSWIFT/qpSWIFT.git
USER root
RUN cd qpSWIFT && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --config Release && \
    cmake --build build --target install
USER ${NB_USER}
RUN cd qpSWIFT/python && python setup.py install

# Build ROS workspace
WORKDIR  ${ROS_WS}
RUN catkin build

# Copy the giskard tmp meshes to skip the converting step
COPY --chown=${NB_USER}:users binder/giskard_decomposed_obj ${ROS_WS}/src/giskardpy/tmp
# Copy contents of the repo into the image
COPY --chown=${NB_USER}:users . /home/${NB_USER}/giskard_examples
WORKDIR /home/${NB_USER}/giskard_examples
RUN git config --global --add safe.directory ${PWD}
RUN ln -s ${ROS_WS} ${PWD}/ROS_WS
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]

COPY --chown=${NB_USER}:users launch/rviz_config/standalone_tracy.rviz ${ROS_WS}/src/giskardpy/launch/rviz_config/standalone_tracy.rviz