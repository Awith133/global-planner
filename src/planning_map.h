//
// Created by Harsh Sharma on 06/10/19.
//

#pragma once
#include <vector>
#include "coordinate.h"

using namespace std;

struct planning_map
{
    vector<vector<double>> map;
    double threshold_elevation_min;
    double threshold_elevation_max;

    planning_map(vector<vector<double>> plan_map,
                 double min_thresh,
                 double max_thresh):
                 map(std::move(plan_map)),threshold_elevation_min(min_thresh),threshold_elevation_max(max_thresh){
    }

    bool is_location_valid(const coordinate &c) const;
    bool is_elevation_valid(const coordinate &c) const;
    bool is_valid(const coordinate &c) const;
};

//=====================================================================================================================
