//
// Created by ray on 19-4-25.
//

#ifndef GLOBAL_PLANNER_STATE_H
#define GLOBAL_PLANNER_STATE_H

#include "Utils.h"
class State{
public:
    double x;
    double y;
    double theta;

    //gx, gy and gtheta are co-ordinates in the 80X80 grid
    double gx;
    double gy;
    double gtheta;

    //for running dijkstra
    int dx;
    int dy;

    float cost2d;
    float cost3d;

    float change;
    float velocity;

    State* next;
    State* previous;

    State();
    State(double x, double y, double theta);
    vector<State> getNextStates(double tolerance);
};


#endif //GLOBAL_PLANNER_STATE_H
