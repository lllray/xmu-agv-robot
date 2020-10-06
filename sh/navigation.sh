#! /bin/sh
#export ROS_MASTER_URI=http://robot:11311
#export ROS_HOSTNAME=robot

source /opt/ros/kinetic/setup.sh
source /home/robot/catkin_ws/devel/setup.sh
source /home/robot/SDK/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.sh
echo "start navigation"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/nav_agv_imu.launch
echo "finish"
