FROM intel4coro/base-notebook:20.04-noetic

# Install VNC server and XFCE desktop environment
USER root
RUN apt-get -y -qq update \
 && apt-get -y -qq install \
        dbus-x11 \
        xfce4 \
        xfce4-panel \
        xfce4-session \
        xfce4-settings \
        xorg \
        xubuntu-icon-theme \
        fonts-dejavu \
    # Disable the automatic screenlock since the account password is unknown
 && apt-get -y -qq remove xfce4-screensaver \
    # chown $HOME to workaround that the xorg installation creates a
    # /home/jovyan/.cache directory owned by root
    # Create /opt/install to ensure it's writable by pip
 && mkdir -p /opt/install \
 && chown -R $NB_UID:$NB_GID $HOME /opt/install \
 && rm -rf /var/lib/apt/lists/*

# Install a VNC server, either TigerVNC (default) or TurboVNC
ENV PATH=/opt/TurboVNC/bin:$PATH
RUN echo "Installing TurboVNC"; \
    # Install instructions from https://turbovnc.org/Downloads/YUM
    wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    gpg --dearmor >/etc/apt/trusted.gpg.d/TurboVNC.gpg; \
    wget -O /etc/apt/sources.list.d/TurboVNC.list https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list; \
    apt-get -y -qq update; \
    apt-get -y -qq install \
        turbovnc \
    ; \
    rm -rf /var/lib/apt/lists/*;

USER $NB_USER
RUN conda install -y websockify
RUN pip install jupyter-remote-desktop-proxy
ENV DISPLAY=:1

# Initiate ROS workspace 
ENV ROS_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${ROS_WS}/src
# A Workaround for fixing the issue of RvizWeb loading meshes with "file:///" path instead of "package:///".
COPY --chown=${NB_USER}:users binder/me ${ROS_WS}/src/me
COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN wstool merge /home/${NB_USER}/noetic.rosinstall \
  && wstool update
# Clone repos with ssh url in .gitmodules
RUN git clone https://github.com/maltehue/mujoco_robots.git \
  && cd mujoco_robots \
  && perl -i -p -e 's|git@(.*?):|https://\1/|g' .gitmodules \
  && git submodule sync \
  && git submodule update --init --recursive
RUN git clone https://github.com/HoangGiang93/mujoco_world.git \
  && cd mujoco_world \
  && perl -i -p -e 's|git@(.*?):|https://\1/|g' .gitmodules \
  && git submodule sync \
  && git submodule update --init --recursive

# Install dependencies
WORKDIR  ${ROS_WS}
USER root
RUN rosdep update && apt update && apt dist-upgrade -y \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions \
  && apt install -y ffmpeg libglfw3 libglfw3-dev
USER ${NB_USER}
RUN pip install -r src/giskardpy/requirements.txt

# Install missing python packages (should be included in src/giskardpy/requirements.txt)
RUN pip install termcolor pydot triangle cython~=0.19

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
# Can not build package iai_pr2_donbot and it is not used
RUN rm -rf src/iai_pr2/iai_pr2_donbot
# Upldate rvizweb
RUN cd src/rvizweb && git pull
RUN catkin build

# Copy the giskard tmp meshes to skip the converting step
COPY --chown=${NB_USER}:users binder/giskard_decomposed_obj ${ROS_WS}/src/giskardpy/tmp
# The giskard standalone does not publish tf messaages by default set it to true for rvizweb
RUN sed -i 's/publish_tf: bool = False/publish_tf: bool = True/g' ${ROS_WS}/src/giskardpy/src/giskardpy/configs/behavior_tree_config.py
# Copy contents of the repo into the image
COPY --chown=${NB_USER}:users . /home/${NB_USER}/giskard_examples
WORKDIR /home/${NB_USER}/giskard_examples
RUN git config --global --add safe.directory ${PWD}
# Soft link the ROS workspace directory to the working directory
RUN ln -s ${ROS_WS} ${PWD}/ROS_WS
# jupyterlab interface configuration
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]

RUN pip install --upgrade jupyter-remote-desktop-proxy