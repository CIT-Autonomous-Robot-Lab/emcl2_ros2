#!/bin/bash

config_path=$(ros2 pkg prefix --share emcl2)/config

ros2 launch emcl2 emcl2.launch.py \
    use_sim_time:=true \
    map:=$config_path/map/map_tsudanuma_campus.yaml \
    params_file:=$config_path/param/emcl2_tsudanuma_campus.param.yaml