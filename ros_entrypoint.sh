#!/bin/bash

# Source the ROS2 installation
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Set up overlay workspace
source "$ROS_WS_PATH/install/setup.bash"
alias build="(cd $ROS_WS_PATH && trap 'cd - > /dev/null' EXIT && \
	colcon build --symlink-install --packages-select-regex turtlebot3)"
alias clean="(cd $ROS_WS_PATH && trap 'cd - > /dev/null' EXIT && \
	rm -rf build/ install/ log/)"

# Add aliases for attaching/detaching the joypad
export JOY_HARDWARE_ID="046d:c21f"
alias attach="sudo modprobe vhci_hcd && sudo usbip --tcp-port 3241 \
    attach --remote localhost --busid \
    \`usbip --tcp-port 3241 list --remote localhost \
    | grep $JOY_HARDWARE_ID | grep -oP '\d+-\d+'\`"
alias detach="sudo usbip --tcp-port 3241 detach --port 0"

# Add aliases for packages
alias bringup="ros2 launch turtlebot3_bringup robot.launch.py"
alias rviz="ros2 launch turtlebot3_rviz2 rviz2.launch.py"
alias teleop="ros2 launch turtlebot3_support turtlebot3_teleop_joy.py"
alias slam="ros2 launch turtlebot3_cartographer cartographer.launch.py"
alias record="(cd $HOME/recordings && trap 'cd - > /dev/null' EXIT && \
	ros2 bag record /tf /tf_static /robot_description /odom /ping)"

# Add compound alias for joypad 
alias joy="(attach && trap 'detach' EXIT && teleop)"

# Some nice colors
export PS1="\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;35m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\\\$ "

exec "$@"
