# Set up DDS for use with Husarnet VPN
export CYCLONEDDS_URI=file://$HOME/husarnet-dds/cyclonedds.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/husarnet-dds/fastdds-simple.xml

# Add aliases for the overlay workspace
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

# Add command for running ros2 commands (including aliases) through docker
ros2() { docker exec -it ros-humble-dev /ros_entrypoint.sh bash -c "ros2 $*"; }