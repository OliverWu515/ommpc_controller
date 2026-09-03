FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG PX4_REF=v1.13.3
ARG OSQP_REF=release-0.6.3

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      bc \
      build-essential \
      geographiclib-tools \
      git \
      gstreamer1.0-plugins-base \
      libgstreamer-plugins-base1.0-dev \
      libeigen3-dev \
      ninja-build \
      python3-catkin-tools \
      python3-matplotlib \
      python3-numpy \
      python3-pip \
      python3-yaml \
      ros-noetic-ddynamic-reconfigure \
      ros-noetic-mavros \
      ros-noetic-mavros-extras \
    && /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "${OSQP_REF}" --recursive \
      https://github.com/osqp/osqp.git /tmp/osqp \
    && cmake -S /tmp/osqp -B /tmp/osqp/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DUNITTESTS=OFF \
    && cmake --build /tmp/osqp/build \
    && cmake --install /tmp/osqp/build \
    && ldconfig \
    && rm -rf /tmp/osqp

RUN git clone --depth 1 --branch "${PX4_REF}" --recursive --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && git -C /opt/PX4-Autopilot/platforms/nuttx/NuttX/nuttx fetch --tags --depth 1 \
    && /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

RUN cd /opt/PX4-Autopilot \
    && DONT_RUN=1 make px4_sitl_default gazebo

RUN mkdir -p /catkin_ws/src

COPY entrypoint.sh /usr/local/bin/ommpc-entrypoint
COPY build_workspace.sh /usr/local/bin/ommpc-build
COPY launch_px4_sitl.sh /usr/local/bin/ommpc-launch-px4
RUN chmod +x /usr/local/bin/ommpc-entrypoint \
             /usr/local/bin/ommpc-build \
             /usr/local/bin/ommpc-launch-px4

WORKDIR /catkin_ws
ENTRYPOINT ["/usr/local/bin/ommpc-entrypoint"]
CMD ["bash"]
