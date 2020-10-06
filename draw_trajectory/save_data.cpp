#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/OccupancyGrid.h"
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <nav_msgs/Odometry.h>
using namespace std;

typedef struct {
    float pose_x_m;
    float pose_y_m;
    float pose_angle;
} RobotPose;
RobotPose origin_pose;
//RobotPose origin_odom_pose;
//RobotPose origin_scan_pose;
float map_resolution=0.0;
ros::Subscriber ros_map_sub;
ros::Subscriber ros_odom_sub;
ros::Publisher ros_scan_odom_pub;

void OdomCallback(const nav_msgs::Odometry& odom) {
    static RobotPose origin_odom_pose{odom.pose.pose.position.x,odom.pose.pose.position.y,0}; 

    ROS_INFO("Received robot pose from odom --- x:%f  y:%f",
             odom.pose.pose.position.x ,
             odom.pose.pose.position.y);
    ofstream outfile("/home/robot/ROS/robot_ws/src/draw_trajectory/data_odom_trajectory.txt",ios::app);

    if (outfile.fail())              // 如果读取失败，打印fail
    {
        cout<< "fail" << endl;
    }
    else{
        cout<<"write successfully!"<<endl;
        outfile<<setprecision(6)<<setw(6)<<odom.pose.pose.position.x-origin_odom_pose.pose_x_m <<" "<<setw(6)<<odom.pose.pose.position.y-origin_odom_pose.pose_y_m<<endl;
        outfile.close();
    }
}

void MapCallback(const nav_msgs::OccupancyGridConstPtr& map) {
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
             map->info.width,
             map->info.height,
             map->info.resolution);
    origin_pose.pose_x_m = map->info.origin.position.x;
    origin_pose.pose_y_m = map->info.origin.position.y;
    map_resolution = map->info.resolution;
    ROS_INFO("Received origin pose --- x:%f  y:%f",
             origin_pose.pose_x_m,
             origin_pose.pose_y_m);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sever");
    ros::NodeHandle nh;
    ros::Rate main_loop_rate(20);
    ros_map_sub = nh.subscribe("/map",1,&MapCallback);
    ros_odom_sub = nh.subscribe("/odom", 30,&OdomCallback);
    ros_scan_odom_pub = nh.advertise<nav_msgs::Odometry>("scan_odom", 30);
    tf::TransformListener tf_pose_listener;

    while(ros::ok()){
         nav_msgs::Odometry odom_;
  odom_.header.frame_id = "scan_odom";
  odom_.child_frame_id = "base_footprint";
        RobotPose *robot_pose = new RobotPose();
            tf::StampedTransform pose_transform;
            try {

                tf_pose_listener.lookupTransform("/map", ros::Time(0),"/base_footprint", // find tf
                                                ros::Time(0),"/odom", pose_transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                //ros::Duration(1.0).sleep();
                main_loop_rate.sleep();
                continue;
            }
        robot_pose->pose_angle=tf::getYaw(pose_transform.getRotation());
    static RobotPose origin_scan_pose{pose_transform.getOrigin().x(),pose_transform.getOrigin().y(),0};
        ofstream outfile("/home/robot/ROS/robot_ws/src/draw_trajectory/data_scan_trajectory.txt",ios::app);

        if (outfile.fail())              // 如果读取失败，打印fail
        {
            cout<< "fail" << endl;
        }
        else{
            cout<<"write successfully!"<<endl;
            outfile<<setprecision(6)<<setw(6)<<pose_transform.getOrigin().x() - origin_scan_pose.pose_x_m<<" "<<setw(6)<<pose_transform.getOrigin().y()-origin_scan_pose.pose_y_m<<endl;
            outfile.close();
        }

        robot_pose->pose_x_m = pose_transform.getOrigin().x() ;
        robot_pose->pose_y_m = pose_transform.getOrigin().y() ;

            ROS_INFO("Received robot pose from map to base_footprint --- x:%f  y:%f",
                     robot_pose->pose_x_m,
                     robot_pose->pose_y_m);

        ros::Time current_time = ros::Time::now();
        odom_.header.stamp = current_time;
        odom_.pose.pose.position.x =  robot_pose->pose_x_m ;
        odom_.pose.pose.position.y =  robot_pose->pose_y_m ;
        odom_.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(robot_pose->pose_angle);
        odom_.pose.pose.orientation = q;
        ros_scan_odom_pub.publish(odom_);


        ros::spinOnce();
        main_loop_rate.sleep();
    }
}
