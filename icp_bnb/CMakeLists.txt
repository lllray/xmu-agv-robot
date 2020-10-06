cmake_minimum_required(VERSION 2.6)
project(icp_bnb)

IF(CMAKE_COMPILER_IS_GNUCC)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  message_generation
  tf
)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
	${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}_lib SHARED
    src/icp.cpp
	include/icp_define.h src/one_means.cpp include/one_means.h)

target_link_libraries(${PROJECT_NAME}_lib
    ${EIGEN3_LIBS}
    ${PCL_LIBRARIES}
)
include_directories(
		include
		${catkin_INCLUDE_DIRS}
)

# TEST ICP
add_executable(icp_odometry test/icp_odometry.cpp)
target_link_libraries(icp_odometry ${catkin_LIBRARIES}
		${PROJECT_NAME}_lib)
#target_link_libraries(icp_odometry
#	${PROJECT_NAME}_lib
#)

