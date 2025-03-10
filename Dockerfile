# syntax=docker/dockerfile:1.7-labs

# ------------------------------------------ Base layer ------------------------------------------
# The base layer contains the ROS2 base packages and additional packages that are commonly used.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

## Install ROS2 additional packages
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

## Also install some useful networking diagnostic tools
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    iputils-ping \
    iputils-clockdiff \
    iputils-arping \
    traceroute \
    mtr-tiny \
    iproute2 \
    nmap \
    tcpdump \
    iperf3 \
    wget \
    curl

## Prepare environment variables for subsequent layers
### TurtleBot3-specific
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=waffle_pi
ENV LDS_MODEL=LDS-01

### DDS provider
ENV CYCLONEDDS_URI=file:///var/lib/husarnet/cyclonedds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/var/lib/husarnet/fastdds-simple.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

### Overlay workspace path
ARG ROS_WS_PATH=/root/ros2_ws
ENV ROS_WS_PATH=${ROS_WS_PATH}

## Update entrypoint
COPY --chmod=0755 ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

## Source setup in subsequent _interactive_ bash shells
RUN printf "\n# ROS2 setup\nsource /ros_entrypoint.sh\n" >> /root/.bashrc

# ---------------------------------------- colcon overlay ----------------------------------------
# This layer provides tools for building custom ROS packages using the colcon build system.
# This is an intermediate layer, and images should be built on top of it, not directly from it.
FROM base AS colcon-overlay

## Install fundamental build tools
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    build-essential

## Install additional colcon extensions, such as Meson and Bazel support
# RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
#     python3-colcon-bazel \
#     python3-colcon-meson

# ---------------------------------------- Device overlay ----------------------------------------
# This layer adds the the low-level drivers and bringup stack, also needed for building the rest.
# This image is intended for development work, and does not feature built-in Husarnet or SSH.
FROM colcon-overlay AS device-overlay

## Install low-level dependencies needed for build (and operation)
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver

# ---------------------------------------- Server overlay ----------------------------------------
# This layer adds the server-side packages to the image, including the navigation stack.
# This image is intended for development work, and does not feature built-in Husarnet or SSH.
FROM colcon-overlay AS server-overlay

## Install server-side packages
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3-simulations

# ---------------------------------------- Build overlay -----------------------------------------
# Add a new layer in which the most recent workspace is pre-compiled. 
# This layer will only be used to copy the pre-compiled workspace into the "Husarnet" images.
FROM device-overlay AS build-overlay

## Create workspace and clone the repository
RUN mkdir -p ${ROS_WS_PATH}/src && git clone https://github.com/nilsjor/turtlebot3.git \
    --sparse \
    --depth=1 \
    --single-branch \
    --branch=humble-devel \
    --filter=blob:none \
    ${ROS_WS_PATH}/src/turtlebot3

## Configure which packages to build using sparse checkout
RUN cd ${ROS_WS_PATH}/src/turtlebot3 && git sparse-checkout set \
    turtlebot3_bringup \
    turtlebot3_cartographer \
    turtlebot3_description \
    turtlebot3_navigation2 \
    turtlebot3_node \
    turtlebot3_rviz2 \
    turtlebot3_support 

## Build the workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${ROS_WS_PATH} && colcon build

# ----------------------------------------- SSH overlay ------------------------------------------
# This layer adds a SSH server with pre-generated keys to the image. 
# The SSH server is configured only allow public key authentication from trusted hosts. 
FROM base AS ssh-overlay

## Define the build argument 
ARG HOSTNAME

## Install SSH server and X11 tools
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    openssh-server \
    xauth \
    x11-apps

## Set SSH authentication methods
COPY sshd_config /etc/ssh/

## Copy pre-generated SSH keys
COPY --chmod=600 --exclude=ssh_keys/*/*.pub ssh_keys/${HOSTNAME}/* /etc/ssh/
COPY --chmod=644 ssh_keys/${HOSTNAME}/*.pub /etc/ssh/

## Copy the public key(s) to authorized_keys
RUN mkdir -p /root/.ssh
COPY --chmod=644 ssh_keys/authorized_keys /root/.ssh/authorized_keys

## Expose SSH port
EXPOSE 22

## Set new entrypoint
COPY ssh_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/ssh_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ssh_entrypoint.sh"]

# --------------------------------------- Husarnet overlay ---------------------------------------
# This layer adds a Husarnet daemon to the image, which will run as the main entrypoint.
# Images based on this layer should only be run *once*, and SSH used to start multiple shells.
# The folder /var/lib/husarnet should be mounted as a volume to persist the Husarnet configuration.
# Additionally, the `husarnet-dds singleshot` service should be run once in a fresh container.
FROM ssh-overlay AS husarnet-overlay

## Use the `HOSTNAME` build argument to set the container's Husarnet hostname
ARG HOSTNAME
ENV HOSTNAME=${HOSTNAME}

## Install Husarnet
RUN apt-get update -y
RUN curl -L https://github.com/husarnet/husarnet/releases/download/v2.0.180/husarnet-linux-$(dpkg --print-architecture).deb -o /tmp/husarnet.deb
RUN apt-get install -y --no-install-recommends --no-install-suggests \
    /tmp/husarnet.deb && rm -f /tmp/husarnet.deb

## Add Husarnet scripts
COPY --chmod=0755 ./husarnet-docker.sh /usr/bin/husarnet-docker
COPY --chmod=0755 ./husarnet-docker-healthcheck.sh /usr/bin/husarnet-docker-healthcheck

## Install Husarnet-DDS
RUN curl -L https://github.com/husarnet/husarnet-dds/releases/download/v1.3.6/husarnet-dds-linux-$(dpkg --print-architecture) -o /usr/local/bin/husarnet-dds
RUN chmod +x /usr/local/bin/husarnet-dds

## Set entrypoint and healthcheck
SHELL ["/usr/bin/bash", "-c"]
HEALTHCHECK --interval=10s --timeout=65s --start-period=5s --retries=6 CMD husarnet-docker-healthcheck || exit 1
CMD husarnet-docker

# ---------------------------------------- Server overlay ----------------------------------------
# This layer adds the server-side packages to the image, including the navigation stack.
# This image is based on the Husarnet overlay, and is intended to be run as a server.
FROM husarnet-overlay AS server-husarnet-overlay

## Install server-side packages
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3-simulations

## Copy the packages build from source
COPY --from=build-overlay \
    --exclude=${ROS_WS_PATH}/install/turtlebot3_cartographer \
    --exclude=${ROS_WS_PATH}/install/turtlebot3_navigation2 \
    --exclude=${ROS_WS_PATH}/install/turtlebot3_rviz2 \
    --exclude=${ROS_WS_PATH}/install/turtlebot3_support \
    ${ROS_WS_PATH}/install ${ROS_WS_PATH}/install
COPY --from=build-overlay /root/.bash_aliases /root/.bash_aliases

# ---------------------------------------- Device overlay ----------------------------------------
# This layer adds the device-side packages to the image; the low-level drivers and bringup stack.
# Images based on this layer are intended to be run as a self-contained Turtlebot3 device, and
# includes the Husarnet overlay for networking. Not sure if this is ever needed...
FROM husarnet-overlay AS device-husarnet-overlay

## Install low-level dependencies
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver

## Copy the packages build from source
COPY --from=build-overlay \
    --exclude=${ROS_WS_PATH}/install/turtlebot3_bringup \
    --exclude=${ROS_WS_PATH}/install/turtlebot3_node \
    ${ROS_WS_PATH}/install ${ROS_WS_PATH}/install
COPY --from=build-overlay /root/.bash_aliases /root/.bash_aliases