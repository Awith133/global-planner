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
#include <unordered_map>

bool is_destination(const coordinate &c, const coordinate &goal);

vector<coordinate> get_true_waypoints(const vector<coordinate> &potential_waypoints,
                                      const vector<vector<double>> &occupancy_map,
                                      const vector<coordinate> &pit_interior);

vector<coordinate> get_neighbors(const int &x,
                                 const int &y,
                                 const vector<vector<double>> &map);

vector<coordinate> get_pit_edges(const vector<vector<double>> &map,
                                 const vector<pair<int,int>> &pit_bbox,
                                 const double &threshold,
                                 vector<coordinate> &pit_interior_points);

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
                                    const unordered_map<coordinate,double,my_coordinate_hasher> &robot_location_heuristic_values,
                                    const rover_parameters &rover_config,
                                    const multi_goal_A_star_configuration &MGA_config);

///
/// This finds the potential goal coordinates, their time_remaining_to_lose_vantage_point_status and the angular distance heuristic values
///
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
                                            const unordered_map<coordinate,double,my_coordinate_hasher> &robot_location_heuristic_values,
                                            const rover_parameters &rover_config);

coordinate get_goal_coordinate_from_lander(const vector<vector<double>> &lit_waypoint_time_data,
                                           const vector<coordinate> &way_points);

coordinate get_pit_center(const vector<coordinate> &waypoints);

unordered_map<coordinate,double,my_coordinate_hasher> get_angle_lookup_table(const vector<coordinate> &way_points,
                                                                             const coordinate &pit_center);      //Returns the angles in radians

int get_last_illuminated_time_step(const vector<vector<double>> &lit_waypoint_time_data);

double get_tentative_robot_angular_change(const double &illumination_start_angle,
                                          const double &illumination_end_angle,
                                          const bool &is_illumination_rotation_clockwise);

unordered_map<coordinate,double,my_coordinate_hasher> get_robot_location_heuristic_values( const vector<coordinate> &goal_coordinates,
                                                                                           const double &tentative_present_robot_angle,
                                                                                           const unordered_map<coordinate,double,my_coordinate_hasher> &angle_lookup_table);

vector<vector<double>> convert_csv_to_vector(const string &file_name);

void convert_vector_to_csv(const vector<coordinate> &way_points,const string &file_name);

vector<coordinate> make_coordinate_vector_from_csv(const string &file_name);

//template <typename T>
void convert_tuple_vector_to_csv(vector<tuple<int,int,int>> time_location,const string &file_name);

string read_text_file(const string &file_name);