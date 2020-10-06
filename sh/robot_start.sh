export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=http://robot:11311
export ROS_HOSTNAME=robot

source /opt/ros/kinetic/setup.sh
source /home/robot/ROS/robot_ws/devel/setup.sh
sleep 1s

roslaunch turtlebot3_bringup turtlebot3_robot.launch 





