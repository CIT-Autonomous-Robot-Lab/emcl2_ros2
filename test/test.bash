#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
source $dir/.bashrc
colcon build
#source $dir/.bashrc
