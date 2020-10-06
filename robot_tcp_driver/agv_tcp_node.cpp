//
// Created by ray on 19-3-31.
//
#include <sever.h>
#include <ros_handle.h>
int main(int argc, char **argv) {
    ros::init(argc, argv, "sever");
    ros::NodeHandle nh;
    ROS_INFO("-------Start!--------");
    if(!SeverInit()) return 1;
    ROS_INFO("SeverInit is success!");
    if(!HandleInit(nh))return 1;
    ROS_INFO("HandleInit is success!");
    ros::Rate main_loop_rate(1000);
    while(ros::ok()){
        handle_spin();
        ros::spinOnce();
        main_loop_rate.sleep();
    }
}