//
// Created by Harsh Sharma on 03/11/19.
//

#include "unit_tests.h"

void test_get_path_to_vantage_point()
{
    vector<coordinate> goal_coordinates; //To be received by the CSPICE illumination function
    goal_coordinates.emplace_back(coordinate{1,13});
    goal_coordinates.emplace_back(coordinate{1,14});
    goal_coordinates.emplace_back(coordinate{2,15});
    goal_coordinates.emplace_back(coordinate{3,15});
    const double MAX_ELEVATION = 100;
    const double MIN_ELEVATION= 90;
    coordinate START_COORDINATE{8,10};
    const rover_parameters rover_config;
    vector<double> time_remaining_to_lose_vantage_point_status{1200,800,600,400}; //To be received by the CSPICE illumination function
    vector<vector<double>> map;     //Define map to test

    assert(time_remaining_to_lose_vantage_point_status.size()==goal_coordinates.size());
    auto func_return = get_path_to_vantage_point(map,MIN_ELEVATION,MAX_ELEVATION+10,START_COORDINATE,goal_coordinates,time_remaining_to_lose_vantage_point_status,rover_config);
    if(!func_return.path.empty())
    {
        cout<<"A path is found"<<endl;
    }
    else
    {
       cout<<"Unable to find path to best goal coordinate "<<endl;
    }
}

//======================================================================================================================