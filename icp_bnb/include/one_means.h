//
// Created by ray on 19-7-5.
//

#ifndef ICP_BNB_ONE_MEANS_H
#define ICP_BNB_ONE_MEANS_H

#include <icp_define.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sensor_msgs/LaserScan.h>
using namespace std;

#define SEARCH_MAX_DISTACCE 2.0
#define SEARCH_ANGLE_OFFSET 30*2
#define SEARCH_MIN_DISTANCE 0.05

vector<Eigen::Vector3d> one_means(sensor_msgs::LaserScan scan);

#endif //ICP_BNB_ONE_MEANS_H
