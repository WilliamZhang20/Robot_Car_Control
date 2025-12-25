#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

# declare mode, use gpu by default
mode="cpu"

# declare sim, use sim by default
sim="True"

while getopts 'ch' opt; do
    case "$opt" in
        c)
            mode="cpu"
            ;;
        ?|h)
            echo "Usage: $(basename $0) [-c]"
            exit 1
            ;;
    esac
done
shift "$(($OPTIND -1))"

export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:/opt/ros/humble/lib/gazebo/plugins"

if [ "$mode" == "gpu" ]; then
    run_docker --runtime=nvidia \
    limo_bot:sim bash
else
    run_docker \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    limo_bot:sim bash
fi