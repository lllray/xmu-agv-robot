//
// Created by ray on 19-7-2.
//

#ifndef LIDAR_SLAM_ICP_DEFINE_H
#define LIDAR_SLAM_ICP_DEFINE_H

#define SCAN_SIZE 270*2

#define PI 3.1415926

#define MAX_DIST 300 //cm

#define CAR_LENGTH 0.70

#define SEED_K 4
#define MAX_M 4

#define FILTER_X_DEFAULT  \
{\
	0.00,1.0,\
	-MAX_DIST/100.0 ,\
	MAX_DIST/100.0 ,\
	{0,0},\
	0,true\
}\

#define FILTER_Y_DEFAULT  \
{\
	0.00,1.0,\
	-MAX_DIST/100.0 ,\
	MAX_DIST/100.0,\
	{0,0},\
	0,true\
}\

#define FILTER_Z_DEFAULT  \
{\
	0.0,1.0,\
	-PI,\
	PI,\
	{0,0},\
	0,true\
}\

typedef struct
{
    const double   former;
    const double   later;
    const double   input_min;
    const double   input_max;
    double Data_Save[2];
    double output;
    bool reset;
}RAY_PRIMARY_FILTER;

#define USE_KD_TREE true

#define DEBUG 0
#define DEBUG_INFO 0
#define SET_MODEL 0
#define READ_MODEL 1
#endif //LIDAR_SLAM_ICP_DEFINE_H
