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

## Install samiemostafavi/perfmeas
RUN apt-get update && apt-get install -y wget
RUN wget https://raw.githubusercontent.com/samiemostafavi/perfmeas/master/pfm -P /usr/local/bin/
RUN chmod +x /usr/local/bin/pfm
RUN sed -i '4i\# Start pfm server\npfm > /proc/1/fd/1 2>&1 &\n' ros_entrypoint.sh

## Install network diagnostics packages
RUN apt-get update && apt-get install -y \
    iputils* \
    traceroute \
    net-tools \
    dnsutils \
    iproute2 \
    nmap \
    tcpdump \
    tcpflow \
    iperf3 \
    curl

# L2TP overlay
FROM netdiag-overlay AS l2tp-overlay

## Set port to be used by L2TP
ENV L2TP_PORT=1701
EXPOSE ${L2TP_PORT}

# Set new entrypoint
COPY l2tp_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/l2tp_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/l2tp_entrypoint.sh"]
CMD ["bash"]

# SSH overlay
FROM l2tp-overlay AS ssh-overlay

## Install SSH server
RUN apt-get update && apt-get install -y \
    openssh-server

## Set SSH authentication methods
COPY sshd_config /etc/ssh/

## Expose SSH port
EXPOSE 22

## Set environment variables for ExPECA
ENV DNS_IP=1.1.1.1
ENV GATEWAY_IP=130.237.11.97

# Set new entrypoint
COPY ssh_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/ssh_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ssh_entrypoint.sh"]
CMD ["bash"]

# Ready-to-go images
## Talker overlay
FROM l2tp-overlay AS talker
CMD ["bash", "-c", "ros2 run demo_nodes_cpp talker"]

## Listener overlay
FROM l2tp-overlay AS listener
CMD ["bash", "-c", "ros2 run demo_nodes_cpp listener"]

## Discovery server overlay
FROM ssh-overlay AS discovery-server
ENV ROS_DISCOVERY_SERVER=127.0.0.1:11811
CMD ["bash", "-c", "fastdds discovery --server-id 0"]

# Desktop overlay
FROM netdiag-overlay AS desktop
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop 
