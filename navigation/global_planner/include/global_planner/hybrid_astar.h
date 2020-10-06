//
// Created by ray on 19-4-25.
//

#ifndef GLOBAL_PLANNER_HYBRID_ASTAR_H
#define GLOBAL_PLANNER_HYBRID_ASTAR_H


#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>
#include "State.h"
#include "Compare.h"
#include "Utils.h"

namespace global_planner {
    class hIndex {
    public:
        hIndex(int a, float b,int t) {
            i = a;
            cost = b;
            theta=t;
        }
        int i;
        float cost;
        int theta;
    };

    struct Previous
    {
        hIndex index[(int)THETA_PER_NUM];
    };

    struct hgreater {
        bool operator()(const hIndex& a, const hIndex& b) const {
            return a.cost > b.cost;
        }
    };

    class HybridAstarExpansion : public Expander {
    public:
        HybridAstarExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                 float* potential){return false;};
        bool calculatePotentialsWithTheta(unsigned char* costs, double start_x, double start_y,const geometry_msgs::PoseStamped& start,\
                                                                                  double end_x, double end_y,const geometry_msgs::PoseStamped& goal,
                                     int cycles, float* potential, double tolerance);
    private:
        int theta_change(int theta){
            if(theta<0)theta+=(int)THETA_PER_NUM;
            if(theta>(int)(THETA_PER_NUM-1))theta-=(int)THETA_PER_NUM;
            return theta;
        }
        int id_change(int i,int theta,int direct ){
            int output=i;
            float e_theta=(theta_change(theta+direct)/THETA_PER_NUM)*2*PI;
            float y=-sin(e_theta)*1.99;//1.99 -> 2
            float x=cos(e_theta)*1.99;
            ROS_INFO("x=%f  y=%f nx=%d",x,y,nx_);
            output+=(int)x + (int)y * nx_;
            return output;
        }
        void add(unsigned char* costs, float* potential, float prev_potential, int next_i,int next_theta, int direct_change,int end_x, int end_y);
        std::vector<hIndex> queue_;
        Previous* previous_array_;
        int getYaw(const geometry_msgs::PoseStamped& input){
            tf::Quaternion quat;
            quat.setX(input.pose.orientation.x);
            quat.setY(input.pose.orientation.y);
            quat.setZ(input.pose.orientation.z);
            quat.setW(input.pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            return (int)((yaw*180/PI)/(360/THETA_PER_NUM));
        }
       // void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
        //std::vector<Index> queue_;
    };

} //end namespace global_planner


#endif //GLOBAL_PLANNER_HYBRID_ASTAR_H
