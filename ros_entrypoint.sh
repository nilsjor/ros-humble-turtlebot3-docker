#!/bin/bash

# Check if the script is *not* running in a Docker container
if [ ! -f /.dockerenv ]; then
    export ROS_DISTRO=humble

    ### TurtleBot3-specific
    export ROS_DOMAIN_ID=30
    export TURTLEBOT3_MODEL=waffle_pi
    export LDS_MODEL=LDS-01

    ### DDS provider
    export CYCLONEDDS_URI=file://$HOME/husarnet-dds/cyclonedds.xml
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/husarnet-dds/fastdds-simple.xml
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    ### Overlay workspace path
    export ROS_WS_PATH=$HOME/ros2_ws

    ### Re-define a few aliases
    alias build="(cd $ROS_WS_PATH && trap 'cd - > /dev/null' EXIT && \
	    colcon build && source install/setup.bash)"
    alias clean="(cd $ROS_WS_PATH && trap 'cd - > /dev/null' EXIT && \
	    colcon clean workspace -y)"
    
    ### Unset wrapper-functions
    unset -f ros2
    unset -f colcon

    ### For development
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
    export TFD_HOME=${HOME}/plansys2/TemporalFastDownward/downward
fi

# Source the ROS2 installation
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the overlay workspace
source "$ROS_WS_PATH/install/setup.bash"

# Run the provided command 
exec "$@"
