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

# Set environment variables
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=waffle_pi
ENV LDS_MODEL=LDS-01

# Install debugging packages
RUN apt-get update && apt-get install -y \
    iputils* \
    net-tools \
    tcpdump \
    traceroute \
    mtr-tiny \
    dnsutils \
    nmap

# Install and configure SSH server
RUN apt-get update && apt-get install -y \
    iproute2 \
    openssh-server

# Set new entrypoint
COPY ssh_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/ssh_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ssh_entrypoint.sh"]
CMD ["/bin/bash"]
