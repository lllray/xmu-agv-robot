//
// Created by ray on 19-4-25.
//

#ifndef GLOBAL_PLANNER_COMPARE_H
#define GLOBAL_PLANNER_COMPARE_H

#include "Utils.h"
#include "State.h"
class Compare
{
public:
    static State target;
    static int** obs_map;
    static int** grid_obs_map;
    static float** shortest_2d;

    bool operator() (const State s1, const State s2);
    float non_holonomic_without_obs(State src);
    float holonomic_with_obs(State src);
    void runDijkstra();
};



#endif //GLOBAL_PLANNER_COMPARE_H
