#! /bin/sh
slam=1
navigation=2
vslam=3
vislam=4
vnavigation=5
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=http://robot:11311
export ROS_HOSTNAME=robot

source /opt/ros/kinetic/setup.sh
source /home/robot/ROS/cartographer_ws/install_isolated/setup.sh
source /home/robot/ROS/robot_ws/devel/setup.sh
source /home/robot/catkin_ws/src/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.sh
source /home/robot/catkin_ws/devel/setup.sh

if [ "$1" = "$navigation" ];then
echo "start navigation"
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_navigation/launch/robot_navigation.launch
elif [ "$1" = "$slam" ];then
echo "start slam"
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_slam/launch/robot_slam.launch
elif [ "$1" = "$vslam" ];then
echo "start vslam"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/Stereo_slam.launch
elif [ "$1" = "$vislam" ];then
echo "start vislam"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/VIStereo_slam.launch
elif [ "$1" = "$vnavigation" ];then
echo "start vnavigation"
roslaunch /home/robot/catkin_ws/src/agv_navigation/launch/nav_agv_imu.launch
fi


