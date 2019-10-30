//
// Created by Harsh Sharma on 06/10/19.
//

#include "planning_map.h"

using namespace std;

bool planning_map::is_location_valid(const coordinate &c) const
{
    return !(c.x < 0 || c.y < 0 || c.x >= map.size() || c.y >= map[0].size());

}

bool planning_map::is_elevation_valid(const coordinate &c) const
{
    return !(map[c.x][c.y] > threshold_elevation_max || map[c.x][c.y] < threshold_elevation_min);

}

bool planning_map::is_valid(const coordinate &c) const
{
    return is_location_valid(c) && is_elevation_valid(c);

}

//=====================================================================================================================
