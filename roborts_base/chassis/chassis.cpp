/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "chassis.h"
#include "../roborts_sdk/sdk.h"
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;

int req_clock_flag=0;
namespace roborts_base{
Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle):
    handle_(handle){
  SDK_Init();
  ROS_Init();
}
Chassis::~Chassis(){
  if(heartbeat_thread_.joinable()){
    heartbeat_thread_.join();
  }
}
void Chassis::SDK_Init(){

  verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>
      (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
       IPC_ADDRESS, CHASSIS_ADDRESS);
  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  verison_client_->AsyncSendRequest(version,
                                    [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                           roborts_sdk::cmd_version_id>::SharedFuture future) {
                                      LOG_INFO << "Chassis Firmware Version: " << int(future.get()->version_id>>24&0xFF) <<"."
                                               <<int(future.get()->version_id>>16&0xFF)<<"."
                                               <<int(future.get()->version_id>>8&0xFF)<<"."
                                               <<int(future.get()->version_id&0xFF);
                                    });
 // robot chassis info sub sdk
  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_ack_push>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_ACK,
                                                           CHASSIS_ADDRESS, IPC_ADDRESS,
                                                           std::bind(&Chassis::ChassisAckInfoCallback, this, std::placeholders::_1));

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_fault_code>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_FALUT_CODE,
                                                             CHASSIS_ADDRESS, IPC_ADDRESS,
                                                             std::bind(&Chassis::ChassisFaultCodeInfoCallback, this, std::placeholders::_1));

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_req_clock>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_REQ_CLOCK,
                                                             CHASSIS_ADDRESS, IPC_ADDRESS,
                                                             std::bind(&Chassis::ChassisReqClockInfoCallback, this, std::placeholders::_1));

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_pose>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_POSE,
                                                             CHASSIS_ADDRESS, IPC_ADDRESS,
                                                             std::bind(&Chassis::ChassisPoseInfoCallback, this, std::placeholders::_1));

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_battery_sta>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_BATTERY_STA,
                                                             CHASSIS_ADDRESS, IPC_ADDRESS,
                                                             std::bind(&Chassis::ChassisBatteryStaInfoCallback, this, std::placeholders::_1));

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_sonar_data>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_SONAR_DATA,
                                                             CHASSIS_ADDRESS, IPC_ADDRESS,
                                                             std::bind(&Chassis::ChassisSonarDataInfoCallback, this, std::placeholders::_1));

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_fault_wdt_push>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_FALUT_WDT,
                                                             CHASSIS_ADDRESS, IPC_ADDRESS,
                                                             std::bind(&Chassis::ChassisFaultWdtPushInfoCallback, this, std::placeholders::_1));

 // robot chassis speed ctl pub sdk
  chassis_ack_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_ack>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_ACK,
                                                                                  IPC_ADDRESS, CHASSIS_ADDRESS);

  chassis_clock_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_clock>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_CLOCK,
                                                                                IPC_ADDRESS, CHASSIS_ADDRESS);

  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                IPC_ADDRESS, CHASSIS_ADDRESS);

  chassis_stop_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_stop>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_STOP,
                                                                                  IPC_ADDRESS, CHASSIS_ADDRESS);

  chassis_req_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_req>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_REQ,
                                                                                 IPC_ADDRESS, CHASSIS_ADDRESS);

  chassis_fault_wdt_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_fault_wdt>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_FALUT_WDT,
                                                                                IPC_ADDRESS, CHASSIS_ADDRESS);

  req_data_thread_ = std::thread(&Chassis::ReqDataThreadHandle,this);

}
void Chassis::ROS_Init(){
  //ros publisher
  ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 30);

  //ros subscriber
  ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);

  //ros_message_init
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";

  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_footprint";

}

void Chassis::ReqDataThreadHandle()
{
    roborts_sdk::cmd_chassis_req chassis_req;
    roborts_sdk::cmd_chassis_ack chassis_ack;
    chassis_req.req_pose=true;
    chassis_req.req_vel=true;
    chassis_req.req_sonar=true;
    chassis_req.req_battery_sta=true;
    chassis_req.req_null=false;
    chassis_ack.ack_value=0x00;// when PIC power on send ack = 0x00 to mcu
    while(ros::ok()){
        if(!req_clock_flag) 
{
  //req_clock_flag=1;
  chassis_ack_pub_->Publish(chassis_ack);
}
        //chassis_req_pub_->Publish(chassis_req);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
void Chassis::ChassisAckInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_ack_push> chassis_info)
{

}

void Chassis::ChassisFaultCodeInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_fault_code> chassis_info)
{
LOG_INFO<<"RUN ChassisFaultCodeInfoCallback:";
LOG_INFO<<"FaultCode"<<(int)chassis_info->fault_code_value;
}

void Chassis::ChassisReqClockInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_req_clock> chassis_info)
{
    req_clock_flag=1;
    if(chassis_info->req_clock_value)
    {
        roborts_sdk::cmd_chassis_clock chassis_clock;
        time_t tt;
        time( &tt );
        tt = tt + 8*3600;  // transform the time zone
        tm* t= gmtime( &tt );
        chassis_clock.year=t->tm_year + 1900;
        chassis_clock.month=t->tm_mon + 1;
        chassis_clock.day=t->tm_mday;
        chassis_clock.hour=t->tm_hour;
        chassis_clock.minute=t->tm_min;
        chassis_clock.sec=t->tm_sec;
        LOG_INFO<<"Times:"<<(int)chassis_clock.year<<"-"<<(int)chassis_clock.month<<"-"<<\
        (int)chassis_clock.day<<"  "<<(int)chassis_clock.hour<<":"<<(int)chassis_clock.minute<<":"<<\
        (int)chassis_clock.sec;
        chassis_clock_pub_->Publish(chassis_clock);
    }
}

void Chassis::ChassisBatteryStaInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_battery_sta> chassis_info){}

void Chassis::ChassisSonarDataInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_sonar_data> chassis_info){}

void Chassis::ChassisFaultWdtPushInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_fault_wdt_push> chassis_info){}


void Chassis::ChassisPoseInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_pose> chassis_info){
 static int time=0;
if(!(time++%200))
{
 LOG_INFO<<"RUN ChassisInfoCallback:";
 LOG_INFO<<chassis_info->position_x_mm<<" "<<chassis_info->position_y_mm<<" "<<chassis_info->position_angle<<" "<<chassis_info->time_ms;
}

  req_clock_flag=1;
  ros::Time current_time = ros::Time::now();
  odom_.header.stamp = current_time;
  odom_.pose.pose.position.x = chassis_info->position_x_mm;
  odom_.pose.pose.position.y = chassis_info->position_y_mm;
  odom_.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info->position_angle);
  odom_.pose.pose.orientation = q;
  odom_.twist.twist.linear.x = chassis_info->v_line;
  //odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
  odom_.twist.twist.linear.y=0;
  odom_.twist.twist.angular.z = chassis_info->v_angle;
  ros_odom_pub_.publish(odom_);

    ofstream outfile("/home/robot/data.txt",ios::app);
/*
    if (outfile.fail())              // 如果读取失败，打印fail
    {
        cout<< "fail" << endl;
    }
    else{
    cout<<"write successfully!"<<endl;
    outfile<<setprecision(6)<<setw(6)<<odom_.twist.twist.linear.x<<" "<<setw(6)<<odom_.twist.twist.angular.z<<"  "<<setw(6)<<odom_.pose.pose.position.x<<" "<<setw(6)<<odom_.pose.pose.position.y<<" "<<setw(6)<<chassis_info->position_angle<<endl;
        outfile.close();
    }
*/

  odom_tf_.header.stamp = current_time;
  odom_tf_.transform.translation.x = chassis_info->position_x_mm;
  odom_tf_.transform.translation.y = chassis_info->position_y_mm;

  odom_tf_.transform.translation.z = 0.0;
  odom_tf_.transform.rotation = q;
 // tf_broadcaster_.sendTransform(odom_tf_);

}

void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel){
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.v_line= vel->linear.x;
  //chassis_speed.vy = vel->linear.y*1000;
  //chassis_speed.v_angle= vel->angular.z * 180/ M_PI;
  chassis_speed.v_angle= vel->angular.z ;
/*
   ofstream outfile("/home/robot/cmdvel_data.txt",ios::app);

    if (outfile.fail())              // 如果读取失败，打印fail
    {
        cout<< "fail" << endl;
    }
    else{
        cout<<"write successfully!"<<endl;
        outfile<<setprecision(6)<<setw(6)<<chassis_speed.v_line<<" "<<setw(6)<< chassis_speed.v_angle<<endl;
        outfile.close();
    }
*/
  chassis_speed_pub_->Publish(chassis_speed);
}

}
