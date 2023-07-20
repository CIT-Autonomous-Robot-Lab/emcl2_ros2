#!/bin/bash -evx

ros2 daemon stop
ros2 daemon start

ros2 launch emcl2 test.launch.xml &
sleep 30

### ESTIMATION RECOVERY TEST ###
# Publish initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
pose:
  pose:
    position: {x: -2.5, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  covariance: [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" --once

# Check if position is within a threshold
ros2 topic echo /mcl_pose --csv | 
head -n 1000 |
awk -F',' '{print $4" "$5}
     sqrt( ($4+2.0)^2 + ($5+0.5)^2 ) < 0.2 {printf "\033[42m%s\033[m\n", "OK";exit(0)}
     NR==1000{printf "\033[41m%s\033[m\n", "TIMEOUT";exit(1)}'

if [ "$?" -ne 0 ]; then
  exit 1
fi

ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap request:\ {}\ 

sleep 10

### NAVIGATION TEST ###
# Publish nav2 goal
ros2 topic pub --qos-history keep_all --qos-reliability reliable --qos-durability transient_local /goal_pose geometry_msgs/msg/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position: {x: -0.5, y: -0.5, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" --once

# Check if position is within a threshold
ros2 topic echo /mcl_pose --csv | 
head -n 1000 |
awk -F',' '{print $4" "$5}
     sqrt( ($4+0.5)^2 + ($5+0.5)^2 ) < 0.3 {printf "\033[42m%s\033[m\n", "OK";exit(0)}
     NR==1000{printf "\033[41m%s\033[m\n", "TIMEOUT";exit(1)}'

ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
killall -9 gzclient gzserver rviz2

if [ "$?" -ne 0 ]; then
  exit 1
fi

RESULT=$?

exit $RESULT
