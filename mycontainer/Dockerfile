ARG ROS_DISTRO=jazzy
FROM docker.io/osrf/ros:${ROS_DISTRO}-desktop-full

# # Fix ROS 2 keys
#RUN rm /etc/apt/sources.list.d/ros2-latest.list \
#    && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
    && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb 
#    && rm -f /tmp/ros2-apt-source.deb

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-dev-tools \
    wget

# Obtain the Nav2 dependencies to build development or distribution workspace against
WORKDIR /root/uros_ws
RUN mkdir -p ~/uros_ws/src
ARG VERSION_TAG=latest
RUN if [ "${ROS_DISTRO}" = "rolling" ]; then \
        git clone https://github.com/ros-planning/navigation2.git --branch main ./src/navigation2 && \
        vcs import ./src/ < ./src/navigation2/tools/underlay.repos; \
    elif [ "${VERSION_TAG}" = "latest" ]; then \
        git clone https://github.com/ros-planning/navigation2.git --branch ${ROS_DISTRO} ./src/navigation2; \
    else \
        git clone https://github.com/ros-planning/navigation2.git --branch ${VERSION_TAG} ./src/navigation2; \
    fi

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init
RUN apt update && apt upgrade -y \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src -r --skip-keys "slam_toolbox turtlebot3_gazebo"

# Get Gazebo Simulator
RUN apt install cppzmq-dev ros-${ROS_DISTRO}-ros-gz -y --no-install-recommends --no-install-suggests || true

# install getting started tools

RUN apt install -y --no-install-recommends --no-install-suggests --allow-unauthenticated \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui

# Add microros
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs ./src/micro_ros_msgs
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent ./src/micro-ROS-Agent

COPY ./microros_misc ./microros_misc
COPY ./src/custom_interfaces ./src/custom_interfaces

RUN apt update && apt upgrade -y --no-install-recommends --no-install-suggests --allow-unauthenticated\
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src -r

# For distribution of Nav2
ARG BUILD=true
ARG COLCON_BUILD_ARGS=""
RUN if [ "${BUILD}" = "true" ]; then \
        . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build $COLCON_BUILD_ARGS; \
    fi

# support venv for python packages
RUN apt install -y python3-venv

WORKDIR /root/nav2_ws

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV MICROROS_DISABLE_SHM=1

RUN chmod +x /root/uros_ws/microros_misc/micro-ros_entrypoint.sh \
&& chmod +x /root/uros_ws/install/setup.bash \
&& chmod +x /opt/ros/$ROS_DISTRO/setup.bash

ENTRYPOINT ["/bin/sh", "/root/uros_ws/microros_misc/micro-ros_entrypoint.sh"]
CMD ["--help"]