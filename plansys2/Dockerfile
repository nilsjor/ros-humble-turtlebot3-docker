# Download base image
FROM ros:humble-ros-base

# Install plansys2 packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    ros-humble-plansys2-bringup \
    ros-humble-plansys2-bt-actions \
    ros-humble-plansys2-core \
    ros-humble-plansys2-domain-expert \
    ros-humble-plansys2-executor \
    ros-humble-plansys2-lifecycle-manager \
    ros-humble-plansys2-msgs \
    ros-humble-plansys2-pddl-parser \
    ros-humble-plansys2-planner \
    ros-humble-plansys2-problem-expert \
    ros-humble-plansys2-terminal

# Install additional ROS 2 packages (requires GUI)
# RUN apt-get update && apt-get install -y \
#     ros-humble-rqt-common-plugins \
#     ros-humble-plansys2-tools
