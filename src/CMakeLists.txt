cmake_minimum_required(VERSION 3.13)
project(Global_Planner)

set(CMAKE_CXX_STANDARD 11)

add_executable(Global_Planner main.cpp coordinate.h coordinate.cpp b_box.h global_map.h global_map.cpp node.h node.cpp planning_map.h planning_map.cpp utility.h utility.cpp map_width_header.h mga_node.h mga_node.cpp rover_parameters.h MGA_config.h unit_tests.h pre_script.cpp)