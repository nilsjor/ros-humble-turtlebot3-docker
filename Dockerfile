# Download base image
FROM ros:humble-ros-base

# Install Cartographer
RUN apt-get update && apt-get install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros

# Install Navigation2
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Install TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3

# Install debugging packages
RUN apt-get update && apt-get install -y \
    iputils* \
    net-tools \
    tcpdump \
    traceroute \
    mtr-tiny \
    dnsutils

# Install SSH server
RUN apt-get update && apt-get install -y \
    openssh-server \
    iproute2

RUN sed -i '4s|^|# First fix the nameserver on the container\n|' /ros_entrypoint.sh
RUN sed -i '5s|^|echo nameserver $DNS_IP > /etc/resolv.conf\n|' /ros_entrypoint.sh
RUN sed -i '6s|^|# Fix the gateway\n|' /ros_entrypoint.sh
RUN sed -i '7s|^|ip route del default\n|' /ros_entrypoint.sh
RUN sed -i '8s|^|ip route add default via $GATEWAY_IP\n|' /ros_entrypoint.sh
RUN sed -i '9s|^|\n|' /ros_entrypoint.sh
