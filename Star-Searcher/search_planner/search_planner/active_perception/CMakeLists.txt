cmake_minimum_required(VERSION 2.8.3)
project(active_perception)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
visualization_msgs
plan_env
bspline
path_searching
cv_bridge
tuw_multi_robot_router
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES active_perception
 CATKIN_DEPENDS plan_env bspline path_searching tuw_multi_robot_router
 DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( active_perception
    src/traj_visibility.cpp 
    src/heading_planner.cpp
    src/frontier_finder.cpp
    src/graph_node.cpp
    src/perception_utils.cpp
    )
target_link_libraries( active_perception
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
