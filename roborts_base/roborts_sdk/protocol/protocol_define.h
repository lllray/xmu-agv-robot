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
#ifndef ROBORTS_SDK_PROTOCOL_DEFINE_H
#define ROBORTS_SDK_PROTOCOL_DEFINE_H

namespace roborts_sdk {

#pragma pack(push, 1)
//DEVICE_ADDRESS
// #define MANIFOLD2_ADDRESS              (0x00u)
#define IPC_ADDRESS                    (0x00u) //dont change
#define CHASSIS_ADDRESS                (0X01u)
#define GIMBAL_ADDRESS                 (0X02u)
#define BROADCAST_ADDRESS              (0Xffu)

//CMD_SET
#define CMD_SET_CONST                  (0x01u)
#define UNIVERSAL_CMD_SET              (0x00u)
#define CHASSIS_CMD_SET                (0x01u)
#define GIMBAL_CMD_SET                 (0x02u)


#define TEST_CMD_SET                   (0xFFu)

/*----------------------------UNIVERSAL_CMD--- 0x00 ---------------------*/
#define CMD_HEARTBEAT                  (0x01u)
typedef struct{
  uint32_t heartbeat;
} cmd_heartbeat;

#define CMD_REPORT_VERSION             (0X02u)
typedef struct
{
  uint32_t version_id;
} cmd_version_id;


/*-----------------------------CHASSIS_CMD---- 0x01 ---------------------*/
#define CMD_SET_CHASSIS_ACK           (0X00u)
typedef struct{
    u_int8_t ack_value;
}cmd_chassis_ack;

#define CMD_SET_CHASSIS_CLOCK           (0X01u)
typedef struct {
    u_int16_t year;
    u_int8_t month;
    u_int8_t day;
    u_int8_t hour;
    u_int8_t minute;
    u_int8_t sec;
}cmd_chassis_clock;

#define CMD_SET_CHASSIS_SPEED          (0X02u)
typedef struct {
    float v_line;
    float v_angle;
} cmd_chassis_speed;

#define CMD_SET_CHASSIS_STOP          (0X03u)
typedef struct {
    u_int8_t stop_value;
} cmd_chassis_stop;

#define CMD_SET_CHASSIS_REQ          (0X04u)
typedef struct {
    u_int8_t req_vel :1;
    u_int8_t req_pose :1;
    u_int8_t req_battery_sta :1;
    u_int8_t req_sonar :1;
    u_int8_t req_null :4;
} cmd_chassis_req;

#define CMD_SET_CHASSIS_FALUT_WDT          (0X05u)
typedef struct {
    u_int8_t fault_wdt_value;
} cmd_chassis_fault_wdt;

#define CMD_PUSH_CHASSIS_ACK          (0X80u)
    typedef struct {
        u_int8_t fault_code_value;
    } cmd_chassis_ack_push;

#define CMD_PUSH_CHASSIS_FALUT_CODE          (0X81u)
typedef struct {
    u_int8_t fault_code_value;
} cmd_chassis_fault_code;

#define CMD_PUSH_CHASSIS_REQ_CLOCK          (0X82u)
typedef struct {
    u_int8_t req_clock_value;
} cmd_chassis_req_clock;

#define CMD_PUSH_CHASSIS_POSE          (0X83u)
typedef struct {
    float v_line;
    float v_angle;
    float position_x_mm;
    float position_y_mm;
    float position_angle;
    u_int32_t time_ms;
} cmd_chassis_pose;

#define CMD_PUSH_CHASSIS_BATTERY_STA          (0X84u)
typedef struct {
    float voltage;
    float quantity;
} cmd_chassis_battery_sta;

#define CMD_PUSH_CHASSIS_SONAR_DATA          (0X85u)
typedef struct {
    float dis1;
    float dis2;
    float dis3;
    float dis4;
} cmd_chassis_sonar_data;

#define CMD_PUSH_CHASSIS_FALUT_WDT          (0X86u)
    typedef struct {
        u_int8_t fault_wdt_value;
    } cmd_chassis_fault_wdt_push;


/*-----------------------------GIMBAL_CMD---- 0x02 ---------------------*/

#define CMD_PUSH_GIMBAL_INFO           (0X01u)
typedef struct {
  uint8_t mode;
  int16_t pitch_ecd_angle;
  int16_t yaw_ecd_angle;
  int16_t pitch_gyro_angle;
  int16_t yaw_gyro_angle;
  int16_t yaw_rate;
  int16_t pitch_rate;
} cmd_gimbal_info;


#define CMD_SET_GIMBAL_MODE            (0X02u)
typedef enum {
  GYRO_CONTROL,
  CODE_CONTROL,
  G_MODE_MAX_NUM,
} gimbal_mode_e;

#define CMD_SET_GIMBAL_ANGLE           (0x03u)
typedef struct{
  union{
    uint8_t flag;
    struct {
      uint8_t yaw_mode:   1;//0 means absolute, 1 means relative;
      uint8_t pitch_mode: 1;
    } bit;
  } ctrl;
  int16_t pitch;
  int16_t yaw;
}cmd_gimbal_angle;




/*-----------------------------TEST_CMD---- 0xFF ---------------------*/
#define TEXT_ECHO_TRANSMIT             (0x00u)
#pragma pack(pop)
}
#endif //ROBORTS_SDK_PROTOCOL_DEFINE_H
