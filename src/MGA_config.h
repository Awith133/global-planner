//
// Created by Harsh Sharma on 28/10/19.
//

#pragma once
#include <utility>

#include "coordinate.h"

struct multi_goal_A_star_configuration
{
    double pessimistic_factor;
    double time_difference_weight;
    explicit multi_goal_A_star_configuration(double pessismistic_time_factor = 2,double weight_time_difference = 2):
            pessimistic_factor(pessismistic_time_factor),time_difference_weight(weight_time_difference){}

};

//=====================================================================================================================

struct multi_goal_A_star_return
{
    double time_to_reach_best_goal;
    vector<coordinate> path;
    multi_goal_A_star_return(double time_taken_to_reach_best_goal,vector<coordinate> backtracked_path):
            time_to_reach_best_goal(time_taken_to_reach_best_goal),path(std::move(backtracked_path)){}

};

//=====================================================================================================================
