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
