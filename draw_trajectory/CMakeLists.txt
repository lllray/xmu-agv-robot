cmake_minimum_required(VERSION 2.8.3)
project(draw_trajectory)

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

add_executable(save_data_node
        save_data.cpp)
target_link_libraries(save_data_node ${catkin_LIBRARIES})
