//
// Created by ray on 19-3-30.
//

#ifndef PROJECT_SEVER_H
#define PROJECT_SEVER_H

#include <zconf.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <string.h>
#include <tcp_define.h>

bool SeverInit();

#endif //PROJECT_SEVER_H
