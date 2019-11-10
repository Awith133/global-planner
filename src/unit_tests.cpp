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

//if(strcmp(argv[1],"around_pit") == 0)
//{
//const int N_ROWS = 20;
//const int N_COLS = 20;
//const double MAX_ELEVATION = 100;
//const double MIN_ELEVATION= 90;
//MAP_LENGTH = N_ROWS;
//MAP_WIDTH = N_COLS;
//global_map g = global_map(N_ROWS,N_COLS,MAX_ELEVATION,MIN_ELEVATION);
//g.display_final_map();
//const auto map = g.g_map;
//const auto pit_bounding_box = g.get_pit_bbox_coordinates();
//const auto threshold = g.get_maximum_elevation()-g.get_minimum_elevation();
//const rover_parameters rover_config;
//
//// TILL HERE IS THE DATA THAT I WILL HAVE ALREADY- THIS INCLUDES THE MAP AND THE PIT BOUNDING BOX
//// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - -
////        vector<coordinate> pit_interior_points;
//auto pit_interior_points = g.get_pit_interior_coordinates();
//const auto pit_edges = get_pit_edges(map,pit_bounding_box,threshold,pit_interior_points);
///// Pit interior needs to be implemented by you! You won't be given this. Just using ground truth as of now
//convert_vector_to_csv(pit_interior_points,"data/trial_pit_interior.csv");
/////Temporary changes made here. Pit interior points are externally given and not obtained in the get_pit_edges function. Revert back.
//
//const int threshold_dist_from_pit{1};
//const double standard_deviation_threshold{.5};
//cout<<"Size of pit_interior_points is"<<pit_interior_points.size()<<endl;
////    auto way_points = generate_way_points(pit_edges,map,threshold_dist_from_pit,pit_interior_points);     //Uncomment this if you want separate waypoints and pit edges
//auto way_points = pit_edges;
//convert_vector_to_csv(way_points,"data/trial_way_points.csv");
//cout<<"Trial Waypoints csv changed"<<endl;
//for(const auto &p: way_points)
//{
//g.way_points.emplace_back(p);
//}
//
//g.display_final_map();
////    g.way_points.clear();
////    auto quarter_waypoints = get_quarter_waypoints(way_points,pit_bounding_box,map,standard_deviation_threshold);
////    for(const auto &quarter_vec: quarter_waypoints)
////    {
////        for(const auto &elt:quarter_vec)
////            g.way_points.emplace_back(elt);
////    }
////    g.display_final_map();
//cout<<"============================================================================="<<endl;
//
/////Path from lander to Pit
////    coordinate start_coordinate{N_ROWS-1,0};
//coordinate start_coordinate{19,0};
////    coordinate goal_coordinate{8,7};
//coordinate goal_coordinate{8,10};
//g.path = get_path(g.g_map,MIN_ELEVATION,MAX_ELEVATION+10,start_coordinate,goal_coordinate);
//g.display_final_map(start_coordinate,goal_coordinate);
//cout<<"============================================================================="<<endl;
/////Old A* Path from Pit Waypoint to Pit waypoint
////    start_coordinate = goal_coordinate;
////    goal_coordinate = g.way_points[0];
////    int way_point_count = 0;
////    while(way_point_count<g.way_points.size())
////    {
////        g.path = get_path(g.g_map,MIN_ELEVATION,MAX_ELEVATION+10,start_coordinate,goal_coordinate);
////        g.display_final_map(start_coordinate,goal_coordinate);
////        cout<<"============================================================================="<<endl;
////        start_coordinate = goal_coordinate;
////        goal_coordinate = g.way_points[++way_point_count];
////    }
//
/////  Multi Goal A* Planning for illuminated coordinates
//way_points = make_coordinate_vector_from_csv("data/trial_way_points.csv");
//cout<<"Waypoints size is: "<<way_points.size()<<endl;
//start_coordinate = goal_coordinate;
//double time_per_step = 700;
//double present_time = 0;
//int present_time_index = 0;
//auto lit_waypoint_time_data = convert_csv_to_vector("data/lit_waypoints.csv");
//cout<<"Number of Lit Waypoints is: "<<lit_waypoint_time_data.size()<<endl;
//assert(lit_waypoint_time_data.size()==way_points.size());
//double final_time_index = lit_waypoint_time_data[0].size();
//unordered_set<coordinate,my_coordinate_hasher> visited_waypoints;
//vector<tuple<int,int,int>> time_location;
//auto previous_coordinate = start_coordinate;
//time_location.emplace_back(make_tuple(present_time_index,previous_coordinate.x,previous_coordinate.y));
//
//while(present_time_index<final_time_index)
//{
//cout<<"At present_time_index: "<<present_time_index<<endl;
//cout<<"Start Position: "<<"\t";
//start_coordinate.print_coordinate();
//cout<<"=================================="<<endl;
//auto start = std::chrono::high_resolution_clock::now();
//vector<double> time_remaining_to_lose_vantage_point_status;
//auto goal_coordinates = get_goal_coordinates(lit_waypoint_time_data,present_time_index,way_points,visited_waypoints,time_per_step,time_remaining_to_lose_vantage_point_status);
//assert(time_remaining_to_lose_vantage_point_status.size()==goal_coordinates.size());
//auto mga_result = get_path_to_vantage_point(g.g_map,MIN_ELEVATION,MAX_ELEVATION+10,start_coordinate,goal_coordinates,time_remaining_to_lose_vantage_point_status,rover_config);
//auto stop = std::chrono::high_resolution_clock::now();
//auto time_taken_to_plan = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
//present_time+= static_cast<double>(time_taken_to_plan.count());
//if(mga_result.path.empty())
//{
//present_time_index +=1;
//cout<<"Path Empty"<<endl;
//time_location.emplace_back(make_tuple(present_time_index,previous_coordinate.x,previous_coordinate.y));
//present_time = present_time_index*time_per_step;            //This line makes the code: 1 point per time frame
//continue;
//}
//present_time+= mga_result.time_to_reach_best_goal;
//g.path = mga_result.path;       ///This needs to be sent to the local planner
//present_time_index = ceil(present_time/time_per_step);
//cout<<"present_time_index: "<<present_time_index<<endl;
//present_time = present_time_index*time_per_step;                //This line makes the code: 1 point per time frame
//const auto best_goal_coordinate = g.path[g.path.size()-1];
//time_location.emplace_back(make_tuple(present_time_index,best_goal_coordinate.x,best_goal_coordinate.y));
//previous_coordinate = best_goal_coordinate;
//visited_waypoints.insert(best_goal_coordinate);
//goal_coordinate = best_goal_coordinate;
////        g.display_final_map(start_coordinate,goal_coordinate);
//start_coordinate = goal_coordinate;
//}
//cout<<"Waypoints visited: "<<visited_waypoints.size()<<endl;
//convert_tuple_vector_to_csv(time_location,"data/time_location_mapping.csv");
//}

//======================================================================================================================