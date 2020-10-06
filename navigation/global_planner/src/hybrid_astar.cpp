//
// Created by ray on 19-4-25.
//

#include "global_planner/hybrid_astar.h"
#include<costmap_2d/cost_values.h>


namespace global_planner {

    HybridAstarExpansion::HybridAstarExpansion(PotentialCalculator *p_calc, int xs, int ys) :
            Expander(p_calc, xs, ys) {
    }

    bool HybridAstarExpansion::calculatePotentialsWithTheta(unsigned char* costs, double start_x, double start_y,const geometry_msgs::PoseStamped& start,\
                                                                                  double end_x, double end_y,const geometry_msgs::PoseStamped& goal,
                                                                                  int cycles, float* potential, double tolerance) {
        int start_theta=getYaw(start);
        int goal_theta=getYaw(goal);
       // previous_array_ = new Previous[nx * ny];
       // State hstart(start_x, start_y, start_theta);

       // State hgoal(end_x, end_y,goal_theta);
        queue_.clear();
        int start_i = toIndex(start_x, start_y);
        queue_.push_back(hIndex(start_i, 0,start_theta));

        std::fill(potential, potential + ns_, POT_HIGH);
        potential[start_i] = 0;

        int goal_i = toIndex(end_x, end_y);
        int cycle = 0;
        ROS_INFO("--------\r\nstart_i  start_x=%f start_y=%f end_x=%f end_y=%f  start_i=%d  goal_i=%d", start_x, start_y,end_x,end_y,start_i,goal_i);
        while (queue_.size() > 0 && cycle < cycles) {
            hIndex top = queue_[0];
            std::pop_heap(queue_.begin(), queue_.end(), hgreater());
            queue_.pop_back();

            int i = top.i;
            if (i == goal_i) {
                ROS_INFO("Reach target!");
                return true;
            }
            int x = i % nx_, y = i / nx_;
ROS_INFO("--------\r\n0  id=%d theta=%d cost=%f  nx=%d x=%d y=%d",top.i,top.theta,top.cost,nx_,x,y);
            add(costs, potential, potential[i], id_change(i,top.theta,0) ,theta_change(top.theta) ,0,end_x, end_y);
            add(costs, potential, potential[i], id_change(i,top.theta,-1) ,theta_change(top.theta-1),1 ,end_x, end_y);
            add(costs, potential, potential[i], id_change(i,top.theta,1) ,theta_change(top.theta+1),1 ,end_x, end_y);
            //add(costs, potential, potential[i], i ,theta_change(top.theta-1) ,end_x, end_y);
            //add(costs, potential, potential[i], i ,theta_change(top.theta+1) ,end_x, end_y);
            cycle++;
        }

        return false;
    }


    void HybridAstarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int next_theta,int direct_change,int end_x,
                             int end_y) {
        if (next_i < 0 || next_i >= ns_)
            return;

        if (potential[next_i]< POT_HIGH)
            return;

        if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
            return;
        potential[next_i] = prev_potential +direct_change*5 + 5;
       //potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
        int x = next_i % nx_, y = next_i / nx_;
        float distance = abs(end_x - x) + abs(end_y - y);
        ROS_INFO("0  id=%d theta=%d cost=%f",next_i,next_theta, potential[next_i] + distance * neutral_cost_);
        queue_.push_back(hIndex(next_i, potential[next_i] + distance * neutral_cost_,next_theta));
        std::push_heap(queue_.begin(), queue_.end(), hgreater());
    }
}//end namespace global_planner
