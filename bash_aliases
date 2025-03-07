# Add aliases for the overlay workspace
alias build="colcon build --symlink-install"
alias clean="colcon clean workspace -y"

# Add aliases for packages
alias bringup="ros2 launch turtlebot3_bringup robot.launch.py"
alias rviz="ros2 launch turtlebot3_rviz2 rviz2.launch.py"
alias teleop="ros2 launch turtlebot3_teleop turtlebot3_teleop_joy.py"
alias slam="ros2 launch turtlebot3_cartographer cartographer.launch.py"

# Add aliases for attaching/detaching the joypad
export JOY_HARDWARE_ID="046d:c21f"
alias attach="sudo modprobe vhci_hcd && sudo usbip --tcp-port 3241 \
    attach --remote localhost --busid \
    \`usbip --tcp-port 3241 list --remote localhost \
    | grep $JOY_HARDWARE_ID | grep -oP '\d+-\d+'\`"
alias detach="sudo usbip --tcp-port 3241 detach --port 0"
alias joy="(attach && trap 'detach' EXIT && teleop)"

# Wrappers for transparently running ros2 commands (including aliases) through docker
ros2() { docker exec -it ros-humble-dev bash ${*:+-ic "ros2 $*"}; }
colcon() { docker exec -it ros-humble-dev bash -ic "colcon $*"; }

# Other aliases for working in CLI mode
alias iperf="iperf3"
alias stop="pkill -SIGINT"