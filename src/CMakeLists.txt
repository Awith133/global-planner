cmake_minimum_required(VERSION 3.14)
project(Create_Global_Waypoints)

set(CMAKE_CXX_STANDARD 11)

add_executable(Create_Global_Waypoints main.cpp coordinate.h coordinate.cpp b_box.h global_map.h global_map.cpp node.h node.cpp planning_map.h planning_map.cpp utility.h utility.cpp map_width_header.h mga_node.h mga_node.cpp rover_parameters.h MGA_config.h)