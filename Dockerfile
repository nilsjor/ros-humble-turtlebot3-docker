# Base Image for TurtleBot3
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

## Install Cartographer
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros

## Install Navigation2
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup

## Install TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3

## Install ros2 demo nodes
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp 

## Set environment variables
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=waffle_pi
ENV LDS_MODEL=LDS-01

## Source setup in subsequent bash shells
RUN printf "\n# ROS2 setup\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n" >> /root/.bashrc

# Networking overlay
FROM base AS netdiag-overlay

## Install network diagnostics packages
RUN apt-get update && apt-get install -y \
    iputils* \
    traceroute \
    net-tools \
    dnsutils \
    iproute2 \
    nmap \
    tcpdump \
    iperf3 \
    curl

# SSH overlay
FROM netdiag-overlay AS ssh-overlay

## Install SSH server
RUN apt-get update && apt-get install -y \
    openssh-server

## Set SSH authentication methods
COPY sshd_config /etc/ssh/

## Expose SSH port
EXPOSE 22

# Set new entrypoint
COPY ssh_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/ssh_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ssh_entrypoint.sh"]
CMD ["bash"]

# Husarnet overlay
FROM ssh-overlay AS husarnet-overlay

RUN apt-get update -y

RUN curl -L https://github.com/husarnet/husarnet/releases/download/v2.0.180/husarnet-linux-amd64.deb -o /tmp/husarnet.deb
RUN apt install -y --no-install-recommends --no-install-suggests \
    /tmp/husarnet.deb && rm -f /tmp/husarnet.deb

COPY --chmod=0755 ./husarnet-docker.sh /usr/bin/husarnet-docker
COPY --chmod=0755 ./husarnet-docker-healthcheck.sh /usr/bin/husarnet-docker-healthcheck

SHELL ["/usr/bin/bash", "-c"]
HEALTHCHECK --interval=10s --timeout=65s --start-period=5s --retries=6 CMD husarnet-docker-healthcheck || exit 1
CMD husarnet-docker

# Ready-to-go images
## Talker overlay
# FROM ssh-overlay AS talker
# CMD ["bash", "-c", "ros2 run demo_nodes_cpp talker"]

# ## Listener overlay
# FROM ssh-overlay AS listener
# CMD ["bash", "-c", "ros2 run demo_nodes_cpp listener"]

# ## Discovery server overlay
# FROM ssh-overlay AS discovery-server
# ENV ROS_DISCOVERY_SERVER=127.0.0.1:11811
# CMD ["bash", "-c", "fastdds discovery --server-id 0"]

# Desktop overlay
FROM netdiag-overlay AS desktop
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop 
