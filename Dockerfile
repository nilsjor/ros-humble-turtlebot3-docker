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

## Set environment variables
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=waffle_pi
ENV LDS_MODEL=LDS-01

# Networking overlay
FROM base AS netdiag-overlay

## Install ros2 demo nodes
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp 

## Install network diagnostics packages
RUN apt-get update && apt-get install -y \
    iputils* \
    net-tools \
    tcpdump \
    traceroute \
    mtr-tiny \
    dnsutils \
    nmap \
    curl \
    wget

# Talker overlay
FROM netdiag-overlay AS talker
CMD ["bash", "-c", "ros2 topic pub /chatter std_msgs/String '{data: \"Hello World\"}' -r 2"]

# Listener overlay
FROM netdiag-overlay AS listener
CMD ["bash", "-c", "ros2 topic echo /chatter"]

# SSH overlay
FROM netdiag-overlay AS ssh-overlay

## Install SSH server and necessary tools
RUN apt-get update && apt-get install -y \
    iproute2 \
    openssh-server

## Set SSH authentication methods
COPY sshd_config /etc/ssh/

## Set environment variables for ExPECA
ENV DNS_IP=1.1.1.1
ENV GATEWAY_IP=130.237.11.97

# Set new entrypoint
COPY ssh_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/ssh_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ssh_entrypoint.sh"]
CMD ["bash"]
