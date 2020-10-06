//
// Created by ray on 19-4-25.
//

#ifndef GLOBAL_PLANNER_UTILS_H
#define GLOBAL_PLANNER_UTILS_H

#include "bits/stdc++.h"
#include <vector>
//#include "opencv/cv.h"
//#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "planner_core.h"

using namespace std;
//using namespace cv;
#define MAPX 800
#define MAPY 800

#define VELOCITY_MAX 60

#define PRIORITY_OBSTACLE_NEAR 10
#define PRIORITY_MOVEMENT 5

#define GX 80
#define GY 80
#define Grid_Res 10

#define DX 240
#define DY 240

#define Theta 72
#define Theta_Res 5

#define BOT_MOVE_DISTANCE 0.25 //40
#define BOT_L 0.5
#define BOT_W 0.5
#define BOT_M_ALPHA 30

#define PI 3.14159

#define THETA_PER_NUM 8.0

#endif //GLOBAL_PLANNER_UTILS_H
