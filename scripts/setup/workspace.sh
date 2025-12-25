#!/bin/bash

cd $WORKSPACE_PATH
source /opt/ros/$ROS_DISTRO/setup.bash
# Forward any arguments to colcon (use --cmake-args for ccache / build flags)
colcon build --symlink-install "$@"
echo "source $WORKSPACE_PATH/install/setup.bash" >> /root/.bashrc