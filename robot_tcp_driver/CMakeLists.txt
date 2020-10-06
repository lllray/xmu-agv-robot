cmake_minimum_required(VERSION 2.8.3)
project(agv_tcp_driver)

IF(CMAKE_COMPILER_IS_GNUCC)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_executable(agv_tcp_node
        server.cpp
        include/sever.h
        agv_tcp_node.cpp
        ros_handle.cpp
        include/tcp_define.h
        include/ros_handle.h)
target_link_libraries(agv_tcp_node ${catkin_LIBRARIES})
