//
// Created by Harsh Sharma on 06/10/19.
//

#pragma once

#include "coordinate.h"
#include "planning_map.h"
#include "rover_parameters.h"
#include "MGA_config.h"
#include <climits>
#include <unordered_set>

bool is_destination(const coordinate &c, const coordinate &goal);

vector<coordinate> astar(const coordinate &start,const coordinate &goal,const planning_map &elevation_map);

//vector<coordinate> implicit_astar(const coordinate &start,
//                                  const coordinate &goal,
//                                  const planning_map &elevation_map,
//                                  const rover_parameters &rover_config);

///
/// INPUT: start location,
///        vector of vantage points,
///        vector of time it'll be illuminated more for,
///         map
///  Have a cost function which takes into account the cost to reach point. The final step to the goal would have a negative cost of (epsilon*t_i - T_i) with a weight factor.
multi_goal_A_star_return multi_goal_astar(const coordinate &start,
                                    const vector<coordinate> &goals,
                                    const planning_map &elevation_map,
                                    const vector<double> &time_remaining_to_lose_vantage_point_status,
                                    const rover_parameters &rover_config,
                                    const multi_goal_A_star_configuration &MGA_config);

vector<coordinate> get_goal_coordinates(const vector<vector<double>> &lit_waypoint_time_data,
                                        const int &present_time_index,
                                        const vector<coordinate> &way_points,
                                        const unordered_set<coordinate,my_coordinate_hasher> &visited_waypoints,
                                        const double &time_per_step,
                                        vector<double> &time_remaining_to_lose_vantage_point_status);

multi_goal_A_star_return get_path_to_vantage_point(const vector<vector<double>> &g_map,
                                            const double &min_elevation,
                                            const double &max_elevation,
                                            const coordinate &start_coordinate,
                                            const vector<coordinate> &goal_coordinates,
                                            const vector<double> &time_remaining_to_lose_vantage_point_status,
                                            const rover_parameters &rover_config);

vector<vector<double>> convert_csv_to_vector(const string &file_name);

void convert_vector_to_csv(const vector<coordinate> &way_points,const string &file_name);

vector<coordinate> make_coordinate_vector_from_csv(const string &file_name);