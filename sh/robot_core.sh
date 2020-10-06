#! /bin/sh
slam=1
navigation_l=2
vslam=3
vislam=4
vnavigation_l=5
vnavigation_m=6
vnavigation_h=7
navigation_m=8
navigation_h=9
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=http://robot:11311
export ROS_HOSTNAME=robot

source /opt/ros/kinetic/setup.sh
source /home/robot/ROS/cartographer_ws/install_isolated/setup.sh
source /home/robot/ROS/robot_ws/devel/setup.sh
source /home/robot/catkin_ws/src/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.sh
source /home/robot/catkin_ws/devel/setup.sh

sleep 1s

if [ "$1" = "$navigation_l" ];then
echo "start navigation"
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_navigation/launch/robot_navigation_l.launch
elif [ "$1" = "$slam" ];then
echo "start slam"
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_slam/launch/robot_slam.launch
elif [ "$1" = "$vslam" ];then
echo "start vslam"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/Stereo_slam.launch
elif [ "$1" = "$vislam" ];then
echo "start vislam"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/VIStereo_slam.launch
elif [ "$1" = "$vnavigation_l" ];then
echo "start vnavigation_l"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/nav_agv_imu_l.launch
elif [ "$1" = "$vnavigation_m" ];then
echo "start vnavigation_m"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/nav_agv_imu_m.launch
elif [ "$1" = "$vnavigation_h" ];then
echo "start vnavigation_h"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/nav_agv_imu_h.launch
elif [ "$1" = "$navigation_m" ];then
echo "start navigation_m"
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_navigation/launch/robot_navigation_m.launch
elif [ "$1" = "$navigation_h" ];then
echo "start navigation_h"
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_navigation/launch/robot_navigation_h.launch
fi


