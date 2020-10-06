export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

source /opt/ros/kinetic/setup.sh
source /home/robot/ROS/cartographer_ws/install_isolated/setup.sh
source /home/robot/ROS/robot_ws/devel/setup.sh
roslaunch /home/robot/ROS/robot_ws/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation_lx.launch 





