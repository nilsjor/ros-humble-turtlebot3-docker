# Download base image
FROM ros:kinetic-ros-base

# Install turtlebot packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    ros-kinetic-turtlebot3-applications \
    ros-kinetic-turtlebot3-applications-msgs \
    ros-kinetic-turtlebot3-automatic-parking \
    ros-kinetic-turtlebot3-automatic-parking-vision \
    ros-kinetic-turtlebot3-autorace \
    ros-kinetic-turtlebot3-autorace-camera \
    ros-kinetic-turtlebot3-autorace-control \
    ros-kinetic-turtlebot3-autorace-core \
    ros-kinetic-turtlebot3-autorace-detect \
    ros-kinetic-turtlebot3-bringup \
    ros-kinetic-turtlebot3-description \
    ros-kinetic-turtlebot3-example \
    ros-kinetic-turtlebot3-fake \
    ros-kinetic-turtlebot3-follower \
    ros-kinetic-turtlebot3-follow-filter \
    ros-kinetic-turtlebot3-gazebo \
    ros-kinetic-turtlebot3-msgs \
    ros-kinetic-turtlebot3-navigation \
    ros-kinetic-turtlebot3-panorama \
    ros-kinetic-turtlebot3-simulations \
    ros-kinetic-turtlebot3-slam \
    ros-kinetic-turtlebot3-teleop

