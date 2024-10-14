# Set up DDS for use with Husarnet VPN
export CYCLONEDDS_URI=file://$HOME/husarnet-dds/cyclonedds.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/husarnet-dds/fastdds-simple.xml

# Add aliases for the overlay workspace
alias build="colcon build --symlink-install"
alias clean="colcon clean packages workspace -y"

# Add aliases for packages
alias bringup="ros2 launch turtlebot3_bringup robot.launch.py"
alias rviz="ros2 launch turtlebot3_rviz2 rviz2.launch.py"
alias teleop="ros2 launch turtlebot3_support turtlebot3_teleop_joy.py"
alias slam="ros2 launch turtlebot3_cartographer cartographer.launch.py"

# Add aliases for attaching/detaching the joypad
export JOY_HARDWARE_ID="046d:c21f"
alias attach="sudo modprobe vhci_hcd && sudo usbip --tcp-port 3241 \
    attach --remote localhost --busid \
    \`usbip --tcp-port 3241 list --remote localhost \
    | grep $JOY_HARDWARE_ID | grep -oP '\d+-\d+'\`"
alias detach="sudo usbip --tcp-port 3241 detach --port 0"

# Add compound alias for joypad
alias joy="(attach && trap 'detach' EXIT && teleop)"

# Add command for running ros2 commands (including aliases) through docker
ros2() { docker exec -it ros-humble-dev /ros_entrypoint.sh bash -c "ros2 $*"; }
colcon() { docker exec -it ros-humble-dev /ros_entrypoint.sh bash -c "colcon $*"; }