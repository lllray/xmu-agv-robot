#! /bin/sh
#export ROS_MASTER_URI=http://robot:11311
#export ROS_HOSTNAME=robot
source /opt/ros/kinetic/setup.sh
source /home/robot/catkin_ws/devel/setup.sh
source /home/robot/catkin_ws/src/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.sh

echo "start mynteye"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/aruco_save.launch
echo "finish"
