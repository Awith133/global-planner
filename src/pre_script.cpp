//
// Created by Harsh Sharma on 03/11/19.
//

//#pragma once

#include <vector>
#include "coordinate.h"
#include "utility.h"

int MAP_WIDTH1;

int temp_main()
{
    string csv_name = "data/globalmap.csv";
    string waypoints_file_name = "data/test_my_waypoints.csv";
    const double test_threshold = 20;
    const vector<pair<int,int>> test_pit_bbox{make_pair(112,110),make_pair(112,145),make_pair(148,110),make_pair(148,145)};
    const auto test_map = convert_csv_to_vector(csv_name);
    MAP_WIDTH1 = test_map[0].size();
    vector<coordinate> test_pit_interior_points;
    const auto test_pit_edges = get_pit_edges(test_map,test_pit_bbox,test_threshold,test_pit_interior_points);
    convert_vector_to_csv(test_pit_edges,waypoints_file_name);
    cout<<"Done"<<endl;
//    //Pit interior point should ideally be an unordered_set. But making it a vector as of now.
//    // This is a design decision. a) There won't be lots of duplication b) The duplication doesn't harm us a lot
    cout<<endl<<"No. of pit interior points: "<<test_pit_interior_points.size()<<endl;
    //
//    const int threshold_dist_from_pit{1};
//    const double standard_deviation_threshold{.5};
//    auto way_points = generate_way_points(pit_edges,test_map,threshold_dist_from_pit,pit_interior_points);
//    const string waypoints_file_name = "/Users/harsh/Desktop/CMU_Sem_3/MRSD Project II/Real_Project_Work/Extra/waypoints.csv";
//    convert_vector_to_csv(way_points,waypoints_file_name);
//

    return 0;
};


