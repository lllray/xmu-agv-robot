#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <icp.h>
#include <eigen3/Eigen/Dense>
#include <icp_define.h>
#include <one_means.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;
using namespace LIDAR;

std::vector<Eigen::Vector3d> pts_model;

bool falg=false;

bool send_flag=false;
bool start_flag=false;
std_msgs::Bool pose_reached_flag;
std_msgs::Bool state_;
ros::Publisher ros_charge_pose_controller_enable_pub;
ros::Publisher ros_charge_pose_controller_disable_pub;
ros::Subscriber ros_agv_state_sub;
ros::Subscriber ros_charge_pose_reached_sub;
Eigen::Vector3d t_output(0.0,0.0,0.0);
double angle_output=0;
Eigen::Quaterniond q_output;
int fail_timses=0;

RAY_PRIMARY_FILTER Filter_X = FILTER_X_DEFAULT;
RAY_PRIMARY_FILTER Filter_Y = FILTER_Y_DEFAULT;
RAY_PRIMARY_FILTER Filter_Z = FILTER_Z_DEFAULT;

//******************************************************//
float Ray_Primary_Filter(double input,RAY_PRIMARY_FILTER *ray)
{
    if(input>ray->input_max){input=ray->input_max;}
    if(input<ray->input_min){input=ray->input_min;}
    ray->Data_Save[1] = input;
    if(ray->reset==false) {
        ray->Data_Save[1] = ray->Data_Save[0] * ray->former + ray->Data_Save[1] * ray->later;
    }else{
        ray->reset=false;
    }
    ray->output=ray->Data_Save[1];
    ray->Data_Save[0]=ray->Data_Save[1];
    return ray->output;

}

void agv_state_callback(const std_msgs::Bool::ConstPtr& state)
{
    state_=*state;
    if(state_.data) {
        std_msgs::String enable;
        enable.data = "base_goal_pose";
        //send pose controller  enable
        ros_charge_pose_controller_enable_pub.publish(enable);
        //start go to charge flag
        start_flag = true;
        ROS_INFO("recevied agv state!");
    }else{
        std_msgs::Empty disable;
        //send pose controller disable
        ros_charge_pose_controller_disable_pub.publish(disable);
        start_flag=false;
    }
}
void pose_reached_callback(const std_msgs::Bool::ConstPtr& msg)
{
    pose_reached_flag.data=msg->data;
    ROS_INFO("recevied pose reached!");
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("recevied scan!");
    if(!(start_flag)&&!(DEBUG)) {
        Filter_X.reset=true;
        Filter_Y.reset=true;
        Filter_Z.reset=true;
        return;
    }

    // Two points clouds.

    std::vector<Eigen::Vector3d> pts_cloud;

    pts_cloud.reserve (scan->ranges.size());

    pts_cloud.clear();

    // Global pose of the camera.
    Eigen::Matrix3d Rwc = Eigen::Matrix3d::Identity();
    Eigen::Vector3d twc ( 0.0, 0.0, 0.0 );
    pts_cloud=one_means(*scan);
    unsigned char * pPixel = 0;

    if(DEBUG_INFO) {
        ROS_INFO("pts_loud.size=%d", pts_cloud.size());
    }

    if(pts_model.size()>0) {
        falg=true;
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t(0.0, 0.0, 0.0);
        Eigen::Vector3d rpy;
        Eigen::Vector3d tar;
        //ROS_INFO("run here!");
       // ICP::findTransformation(pts_model, pts_cloud, 10, 1.0e-5, 1.0e-3, R, t);

        if(ICP::findTransformation(pts_model, pts_cloud, 3, 1.0e-5, 1.0e-3, R, t,tar)) {
            t_output[1] = -Ray_Primary_Filter(tar[0], &Filter_X);
            t_output[0] = Ray_Primary_Filter(tar[1], &Filter_Y);
            t_output[2] = t[2];
            rpy = R.eulerAngles(0, 1, 2);
            angle_output = -Ray_Primary_Filter(rpy[2], &Filter_Z);
            ROS_INFO("success!");
            ROS_INFO("TARGET:%f  %f  %f", t_output[0],  t_output[1], angle_output);
            send_flag = true;
fail_timses=0;
        }
        else{
fail_timses++;
            ROS_ERROR("get pose fail!!!!!");
if(fail_timses>=3){
        std_msgs::Empty disable;
                //send pose controller disable
                ros_charge_pose_controller_disable_pub.publish(disable);
                // cancel process}
                send_flag=false;
                start_flag=false;
                pose_reached_flag.data = false;
                state_.data=false;
}
        }


        // save poses to file.
        if (DEBUG_INFO) {
            ofstream outfile("/home/robot/data_output.txt", ios::app);
            if (outfile.fail()) {
                cout << "fail" << endl;
            } else {

                cout << "write successfully!" << endl;
                outfile << setprecision(6) << " " << setprecision(9) << ros::Time::now () <<" "<< tar[0] << " " << tar[1] << " " << rpy[2]
                        <<" "<< t_output[0] << " " << t_output[1] << " " << angle_output << endl;
                // print poses.
                std::cout << setprecision(6) << " " << setprecision(9) << ros::Time::now () <<" "<< tar[0] << " " << tar[1] << " " << rpy[2]
                        <<" "<< t_output[0] << " " << t_output[1] << " " << angle_output << endl;
            }
        }
    }else{
         if(READ_MODEL) {

             ifstream infile("/home/robot/Map/v_model.txt");
             Eigen::Vector3d data;
             while (infile >> data[0]) {
                 infile >> data[1]; //   infile>>data[0] ;  //读数据的时候因为数据间有一个空格才能完整的读出
                 infile >> data[2];
                 cout << data[0] << " " << data[1] << " " << data[2] << endl;  //输出到屏幕，加上空格是为了分开数字
                 pts_model.push_back(data);

             }

         }
         else if(SET_MODEL) {
            pts_model.assign(pts_cloud.begin(),pts_cloud.end());
            for (int i = 0; i < pts_model.size(); i++) {
                ofstream outfile("/home/robot/v_model.txt", ios::app);
                if (outfile.fail()) {
                    cout << "fail" << endl;
                } else {

                    cout << "write successfully!" << endl;
                    outfile << setprecision(6) << " " << setprecision(9)  << pts_model[i][0]
                            << " "<< pts_model[i][1]-0.4 << " " << pts_model[i][2]<< endl;
                    // print poses.
                    std::cout<< setprecision(6) << " " << setprecision(9)  << pts_model[i][0]
                            << " "<< pts_model[i][1]-0.4 << " " << pts_model[i][2]<< endl;
                }
            }
        }else{
             pts_model.assign(pts_cloud.begin(),pts_cloud.end());
         }
        ROS_INFO("GET pts_model %d:",pts_model.size());
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "icp_test");
    ros::NodeHandle n;

    ros::Subscriber scan_sub_;
    scan_sub_ = n.subscribe<sensor_msgs::LaserScan>("/scan",1, &scan_callback);

    ros_agv_state_sub=n.subscribe<std_msgs::Bool>("/agv_state",1,&agv_state_callback);

    ros_charge_pose_reached_sub=n.subscribe<std_msgs::Bool>("/diff_drive_pose_controller/pose_reached",1,&pose_reached_callback);

    ros_charge_pose_controller_enable_pub=n.advertise<std_msgs::String>("/diff_drive_pose_controller/enable",1);

    ros_charge_pose_controller_disable_pub=n.advertise<std_msgs::Empty>("/diff_drive_pose_controller/disable",1);
    pts_model.reserve (SCAN_SIZE);
    ros::Rate r(50);

    static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    tf::TransformBroadcaster broadcaster;
    ROS_INFO("start!");

    while(n.ok()) {
        if (DEBUG) {
            ros::spinOnce();
            r.sleep();
            continue;
        }
        //start_flag=true;
        ROS_INFO("goto_charge_state=%d, start_flag=%d, send_flag=%d, pose_reached_flag.data=%d",(int)state_.data,(int)start_flag,(int)send_flag,(int)(bool)pose_reached_flag.data);
        //if start and v was detection
        if(start_flag&&send_flag){
            //if arrive charge goal
            if(pose_reached_flag.data)
            {
                std_msgs::Empty disable;
                //send pose controller disable
                ros_charge_pose_controller_disable_pub.publish(disable);
                // cancel process
                send_flag=false;
                start_flag=false;
                pose_reached_flag.data = false;
                state_.data=false;
            }
            else {
                tf::Quaternion q;
                q = tf::createQuaternionFromYaw(angle_output);
                broadcaster.sendTransform(
                        tf::StampedTransform(
                                tf::Transform(q, tf::Vector3(t_output[0],t_output[1] , 0.0)),
                                ros::Time::now(), "base_scan", "base_goal_pose"));

                send_flag = false;
                ROS_INFO("get goal sucess");
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}
