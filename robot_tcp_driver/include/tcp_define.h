//
// Created by ray on 19-3-31.
//
#include <stdlib.h>
#ifndef AGV_TCP_DRIVER_TCP_DEFINE_H
#define AGV_TCP_DRIVER_TCP_DEFINE_H

#define HEAD_SOF 0xAA
#define HEAD_TAIL 0x77
typedef struct{
    uint8_t sof;
    uint8_t id;
    uint8_t len;
}CmdHead;

typedef struct {
    CmdHead head;
    uint8_t data[256]; //if cmd data
    void *  data_ptr;  //if map or large data
    uint8_t  tail;
} CmdMessage;

enum DirectCmd{
    UP = 1,
    DOWN,
    LEFT,
    RIGHT,
    STOP
};

/*-----------------CMD_REQ_MAP----0x01---------------*/
#define CMD_REQ_MAP (0x01)
typedef struct {
    int8_t req_value;  //
} ReqMap;

/*---------------CMD_SET_AGV_SPEED----0x02-----------*/
#define CMD_SET_AGV_SPEED (0x02)
typedef struct {
    int8_t v_angle; //x -127~+127
    int8_t v_line;  //y -127~+127
} SetAgvSpeed;

/*---------------CMD_SET_SPEED_RANGE----0x03---------*/
#define CMD_SET_SPEED_RANGE (0x03)
#define MAX_CMD_V_LINE 1.0
#define MAX_CMD_V_ANGLE 1.0
typedef struct {
    int8_t max_v_line;
    int8_t max_v_angle;
} SetSpeedRange;

/*---------------CMD_SET_GOAL_POSE----0x04-----------*/
#define CMD_SET_GOAL_POSE (0x04)
typedef struct {
    float pose_x_m;
    float pose_y_m;
    float pose_angle;
} SetGoalPose;

/*---------------CMD_SEND_ROBOT_POSE----0x80---------*/
#define CMD_SEND_ROBOT_POSE (0x80)
typedef struct {
    float pose_x_m;
    float pose_y_m;
    float pose_angle;
} SendRobotPose;

/*------------------CMD_SEND_MAP----0xff-------------*/
#define CMD_SEND_MAP (0xff)
#define MAP_SIZE_RANGE 1024
union MapHeadUnion
{
    uint32_t at;
    uint8_t tab[4];
};
typedef struct{
    MapHeadUnion width;
    MapHeadUnion height;
    SetGoalPose origin_pose;
}MapHead;
typedef struct {
    MapHead map_head;
    void *  data_ptr;
} MapMessage;

bool Take(CmdMessage *recv_container);
bool Send(CmdMessage &send_container);

#endif //AGV_TCP_DRIVER_TCP_DEFINE_H
