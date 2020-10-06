#! /bin/sh
export ROS_MASTER_URI=http://robot:11311
export ROS_HOSTNAME=robot

source /opt/ros/kinetic/setup.sh
source /home/robot/ROS/cartographer_ws/install_isolated/setup.sh
source /home/robot/ROS/robot_ws/devel/setup.sh

rosservice call /write_state "filename: 'map.pbstream' 
include_unfinished_submaps: true" 
cp /home/robot/.ros/map.pbstream /home/robot/Map
rosrun map_server map_saver -f /home/robot/Map/map

