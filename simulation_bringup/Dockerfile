FROM ros:noetic
ARG WS=/opt/ros/overlay_ws


# Install dependencies
RUN apt update --fix-missing -y
RUN apt install -y \
    build-essential cmake git libpoco-dev libeigen3-dev \
    ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon

# Install packages
RUN apt install -y \
    ros-noetic-libfranka ros-noetic-gazebo-ros

# Build external packages
WORKDIR $WS
RUN catkin init
WORKDIR $WS/src
RUN git clone -b noetic-devel https://github.com/ros-planning/panda_moveit_config.git
RUN git clone -b develop https://github.com/frankaemika/franka_ros.git
RUN git clone -b master https://github.com/ros-planning/moveit.git
WORKDIR $WS
RUN rosdep update
RUN catkin config --extend /opt/ros/noetic
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -y \
      --from-paths src\
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*
RUN catkin build panda_moveit_config
RUN catkin build franka_ros
RUN catkin build moveit


# Build source
WORKDIR $WS/src
COPY . ./simulation_bringup
WORKDIR $WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -y \
      --from-paths src\
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build simulation_bringup


# Source entrypoint setup
ENV WS $WS
RUN sed --in-place --expression \
      '$isource "$WS/devel/setup.bash"' \
      /ros_entrypoint.sh

# Run launch file
CMD roslaunch simulation_bringup bringup.launch
