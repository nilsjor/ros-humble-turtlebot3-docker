#!/bin/bash

# Source the ROS2 installation
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the overlay workspace
source "$ROS_WS_PATH/install/setup.bash"

# Run the provided command 
exec "$@"
