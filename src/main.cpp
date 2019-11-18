#include <iostream>
#include <vector>
#include <iomanip>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include "unit_tests.h"
//#include "convert_img_to_map.h"
#include "coordinate.h"
#include "b_box.h"
#include "global_map.h"
#include "planning_map.h"
#include "utility.h"
#include "rover_parameters.h"

using namespace std;

//=====================================================================================================================
/// Global variables

int MAP_WIDTH;

//=====================================================================================================================

struct coordinate_hasher
{
    size_t
    operator()(const coordinate &obj) const
    {
        return std::hash<int>()(obj.x * MAP_WIDTH + obj.y);
    }
};

//=======================================================================================================================

void dfs_util(unordered_set<coordinate,coordinate_hasher> &accept_list,
              unordered_set<coordinate,coordinate_hasher> &reject_list,
              unordered_set<coordinate,coordinate_hasher> &visited,
              const int &depth,
              const vector<vector<double>> &map,
              const coordinate &present_coordinate,
              const int &threshold)
{
    visited.insert(present_coordinate);
    const auto neighbors = get_neighbors(present_coordinate.x,present_coordinate.y,map);
    for(const auto &neighbor:neighbors)
    {
        //cout<<"=================================================="<<endl;
        //neighbor.print_coordinate();

        if(reject_list.count(neighbor)!=0)
        {
//            cout<<"Vertex already in reject list"<<endl;
            continue;
        }

        if(depth+1==threshold)
        {
            if(accept_list.count(neighbor)==0)
                {
                    accept_list.insert(neighbor);
//                    cout<<"Adding vertex to accept_list"<<endl;
                }
            continue;
        }

        if(visited.count(neighbor)==0)
        {
            reject_list.insert(neighbor);
//            cout<<"Adding to reject list"<<endl;
            if(accept_list.count(neighbor)!=0)
            {
                accept_list.erase(neighbor);
//                cout<<"Vertex removed from accept list"<<endl;
            }
            if(depth+1<threshold)
                dfs_util(accept_list,reject_list,visited,depth+1,map,neighbor,threshold);
        }
        else
        {
            cout<<"Already Visited"<<endl;
        }
    }
}

//=======================================================================================================================

vector<coordinate> generate_way_points(const vector<coordinate> &pit_edges,
                                        const vector<vector<double>> &map,
                                        const int &threshold,
                                        const vector<coordinate> &pit_interior)
{
    unordered_set<coordinate,coordinate_hasher> accept_list;
    unordered_set<coordinate,coordinate_hasher> reject_list;
    for(const auto &pit_edge:pit_edges)
    {
        reject_list.insert(pit_edge);
        //pit_edge.print_coordinate();
    }

    for(const auto &coordinate_in_pit:pit_interior)
    {
        reject_list.insert(coordinate_in_pit);
        //coordinate_in_pit.print_coordinate();
    }

    for(auto const &pit_edge: pit_edges)
    {
        unordered_set<coordinate,coordinate_hasher> visited;
        int depth = 0;
        dfs_util(accept_list,reject_list,visited,depth,map,pit_edge,threshold);
    }
    cout<<"================================"<<endl;
    vector<coordinate> way_points;
    way_points.reserve(accept_list.size());
    //cout<<"Size of way points: "<<accept_list.size()<<endl;
    for(const auto &elt:accept_list)
    {
        way_points.push_back(elt);
//        elt.print_coordinate();
    }

    return std::move(way_points);
}

//======================================================================================================================

vector<coordinate> get_feasible_waypoints(const unordered_set<coordinate,coordinate_hasher> &this_quarter_waypoints,
                                          const vector<vector<double>> &map,
                                          const double mean_elevation,
                                          const double &standard_deviation_threshold)
{
    vector<coordinate> result;
    double  standardDeviation=0;
    for(const auto &elt:this_quarter_waypoints)
        standardDeviation += pow(map[elt.x][elt.y] - mean_elevation, 2);
    standardDeviation = pow(standardDeviation, 0.5);
    cout<<"Higher bound: "<<mean_elevation+(standard_deviation_threshold*standardDeviation)<<endl;
    cout<<"Lower bound: "<<mean_elevation-(standard_deviation_threshold*standardDeviation)<<endl;

    for(const auto &elt:this_quarter_waypoints)
    {
//        cout<<"Elevation at this point is: "<<map[elt.x][elt.y]<<endl;
        if(map[elt.x][elt.y]<mean_elevation+(standard_deviation_threshold*standardDeviation) &&
                map[elt.x][elt.y]>mean_elevation-(standard_deviation_threshold*standardDeviation))
        {
            result.push_back(elt);
//            cout<<"Acccepted"<<endl;
        }
    }
    return std::move(result);
}
//======================================================================================================================

vector<vector<coordinate>> get_quarter_waypoints(const vector<coordinate> &possible_waypoints,
                      const vector<pair<int,int>> &pit_bbox,
                      const vector<vector<double>> &map,
                      const double standard_deviation_threshold
                      )
{
    cout<<standard_deviation_threshold<<endl;
    bbox b(0,0,0,0);
    b.get_bbox_coord(pit_bbox);
    const auto mid_x = b.get_mid_x();
    const auto mid_y = b.get_mid_y();
    unordered_set<coordinate,coordinate_hasher> possible_waypoints_set;
    for(auto x:possible_waypoints)
    {
        possible_waypoints_set.insert(x);
    }
    vector<vector<coordinate>> result;
    unordered_set<coordinate,coordinate_hasher> this_quarter_waypoints;
    double elevation_sum = 0;
    int num_elt = 0;
    //Quarter 1 Waypoints
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x<=mid_x && it->y<=mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));

    //Quarter 2 Waypoints
    this_quarter_waypoints.clear();
    elevation_sum = 0;
    num_elt = 0;
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x<=mid_x && it->y>mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));

    //Quarter 3 Waypoints
    this_quarter_waypoints.clear();
    elevation_sum = 0;
    num_elt = 0;
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x>mid_x && it->y>=mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));

    //Quarter 4 Waypoints
    this_quarter_waypoints.clear();
    elevation_sum = 0;
    num_elt = 0;
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x>mid_x && it->y<mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));
    return std::move(result);
}

//======================================================================================================================

vector<coordinate> get_path(const vector<vector<double>> &g_map,
                            const double &min_elevation,
                            const double &max_elevation,
                            const coordinate &start_coordinate,
                            const coordinate goal_coordinate)
{
    planning_map my_map{g_map,min_elevation,max_elevation}; //Pit interiors have to be made obstacle here. Tune min elevation according to that
    return astar(start_coordinate,goal_coordinate,my_map);
}

//======================================================================================================================

int main(int argc, char** argv) {

    if(strcmp(argv[1],"around_pit") == 0)
    {
        const double MIN_TRAVERSABLE_ELEVATION = 0.5;
        const double MAX_TRAVERSABLE_ELEVATION = 1.5;
        const rover_parameters rover_config;

        //Reading Files
        auto pit_interior_points = make_coordinate_vector_from_csv("data/pit_interior.csv");
        const auto way_points = make_coordinate_vector_from_csv("data/waypoints.csv");
        const auto map = convert_csv_to_vector("data/occupancy_global_map.csv");
        const auto start_quadrant_index = stoi(read_text_file("data/illumination_start_quadrant.txt"));
        auto lit_waypoint_time_data = convert_csv_to_vector("data/lit_waypoints.csv");
        const auto illumination_endpoint_index = stoi(read_text_file("data/pit_info.txt"));
        bool is_illumination_rotation_clockwise = true; //To be read later

        MAP_WIDTH = map[0].size();

        cout<<"Number of Lit Waypoints is: "<<lit_waypoint_time_data.size()<<endl;
        cout<<"start_quadrant_index: "<<start_quadrant_index<<endl;

//      const int threshold_dist_from_pit{1};
//      const double standard_deviation_threshold{.5};
//      auto way_points = generate_way_points(pit_edges,map,threshold_dist_from_pit,pit_interior_points);     //Uncomment this if you want separate waypoints and pit edges

        const unordered_map<int,coordinate> start_positions {{1,coordinate{0,0}},
                                                             {2,coordinate{0,208}},
                                                             {3,coordinate{static_cast<int>(map.size()-1),static_cast<int>(map[0].size()-1)}},
                                                             {4,coordinate{static_cast<int>(210),50}}};


        coordinate start_coordinate = start_positions.at(start_quadrant_index);
        start_coordinate.print_coordinate();

///     Lander to Pit Traversal
        auto goal_coordinate = get_goal_coordinate_from_lander(lit_waypoint_time_data,way_points);
        cout<<"Goal coordinate: "<<endl;
        goal_coordinate.print_coordinate();
        const auto trajectory = get_path(map,MIN_TRAVERSABLE_ELEVATION,MAX_TRAVERSABLE_ELEVATION,start_coordinate,goal_coordinate);
        cout<<"Path_Length: "<<trajectory.size()<<endl;

        //Writing data to CSV's
        const string trajectory_file_name = "data/lander_to_pit_trajectory.csv";
        convert_vector_to_csv(trajectory,trajectory_file_name);

///     Pre-process data for Multi Goal A*
        const auto pit_center = get_pit_center(way_points);
        pit_center.print_coordinate();
        const auto angle_lookup_table = get_angle_lookup_table(way_points,pit_center);
        const auto illumination_start_angle = angle_lookup_table.at(goal_coordinate);
        const auto last_illuminated_waypoint  = way_points[illumination_endpoint_index];
        const auto illumination_end_angle = get_angle(pit_center,last_illuminated_waypoint);
        const auto tentative_robot_angular_change = get_tentative_robot_angular_change(illumination_start_angle,illumination_end_angle,is_illumination_rotation_clockwise);
        const auto total_lit_time = 1 + get_last_illuminated_time_step(lit_waypoint_time_data);
        const auto tentative_robot_angular_velocity = tentative_robot_angular_change/total_lit_time;
//        cout<<"get_tentative_robot_angular_change "<<tentative_robot_angular_change<<endl;
//        cout<<"total_lit_time "<<total_lit_time<<endl;
//        cout<<"tentative_robot_angular_velocity "<<tentative_robot_angular_velocity<<endl;
//        cout<<"illumination_start_angle "<<illumination_start_angle<<endl;
        cout<<"illumination_end_angle "<<illumination_end_angle<<endl;

///  Multi Goal A* Planning for illuminated coordinates
        double time_per_step = 700;
        double present_time = 0;
        int present_time_index = 0;
        assert(lit_waypoint_time_data.size()==way_points.size());
        double final_time_index = lit_waypoint_time_data[0].size();
        unordered_set<coordinate,my_coordinate_hasher> visited_waypoints;
        vector<tuple<int,int,int,int>> time_location;
        auto previous_coordinate = goal_coordinate;
        int intermediate_waypoint_label = 0;
        int final_waypoint_label = 1;
        int dont_repeat_waypoint_label = -1;
        time_location.emplace_back(make_tuple(present_time_index,previous_coordinate.x,previous_coordinate.y,final_waypoint_label));

        while(present_time_index<final_time_index)
        {
//            cout<<"At present_time_index: "<<present_time_index<<endl;
//            cout<<"Start Position: "<<"\t";
//            start_coordinate.print_coordinate();
//            cout<<"=================================="<<endl;
            auto start = std::chrono::high_resolution_clock::now();
            auto tentative_present_robot_angle = get_tentative_present_robot_angle(illumination_start_angle,tentative_robot_angular_velocity,present_time_index,is_illumination_rotation_clockwise);
            vector<double> time_remaining_to_lose_vantage_point_status;
            auto goal_coordinates = get_goal_coordinates(lit_waypoint_time_data,present_time_index,way_points,visited_waypoints,time_per_step,time_remaining_to_lose_vantage_point_status);
            const auto present_time_step_heuristic_value = get_robot_location_heuristic_values(goal_coordinates,tentative_present_robot_angle,angle_lookup_table);
            //assert(time_remaining_to_lose_vantage_point_status.size()==goal_coordinates.size());
            auto mga_result = get_path_to_vantage_point(map,MIN_TRAVERSABLE_ELEVATION,MAX_TRAVERSABLE_ELEVATION,start_coordinate,goal_coordinates,time_remaining_to_lose_vantage_point_status,present_time_step_heuristic_value,rover_config);
            auto stop = std::chrono::high_resolution_clock::now();
            auto time_taken_to_plan = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
            present_time+= static_cast<double>(time_taken_to_plan.count());
            if(mga_result.path.empty())
            {
                present_time_index +=1;
                cout<<"Path Empty"<<endl;
                time_location.emplace_back(make_tuple(present_time_index,previous_coordinate.x,previous_coordinate.y,dont_repeat_waypoint_label));
                present_time = present_time_index*time_per_step;            //This line makes the code: 1 point per time frame
                continue;
            }
            present_time+= mga_result.time_to_reach_best_goal;
//            g.path = mga_result.path;       ///This needs to be sent to the local planner
            present_time_index = ceil(present_time/time_per_step);
            cout<<"present_time_index: "<<present_time_index<<endl;
            present_time = present_time_index*time_per_step;                //This line makes the code: 1 point per time frame
            const auto best_goal_coordinate = mga_result.path[mga_result.path.size()-1];
            for(const auto &point:mga_result.path)
            {
                if(point!=best_goal_coordinate)
                    time_location.emplace_back(make_tuple(present_time_index,point.x,point.y,intermediate_waypoint_label));
                else
                    time_location.emplace_back(make_tuple(present_time_index,point.x,point.y,final_waypoint_label));
            }
            previous_coordinate = best_goal_coordinate;
            visited_waypoints.insert(best_goal_coordinate);
            start_coordinate = best_goal_coordinate;
        }
        cout<<"Waypoints visited: "<<visited_waypoints.size()<<endl;
        convert_tuple_vector_to_csv(time_location,"data/time_location_mapping.csv");
    }

    else if(strcmp(argv[1],"lander_to_pit") == 0)
    {
        ///Testing on real global image
        string elevation_map_file = "data/elevation_global_map.csv";
        const auto occupancy_map = convert_csv_to_vector("data/occupancy_global_map.csv");
        const double ELEVATION_THRESHOLD = 20;
        const int threshold_dist_from_pit{1};
        const vector<pair<int,int>> test_pit_bbox{make_pair(112,110),make_pair(112,145),make_pair(148,110),make_pair(148,145)};
        const auto elevation_map = convert_csv_to_vector(elevation_map_file);
        MAP_WIDTH = elevation_map[0].size();
        vector<coordinate> test_pit_interior_points;
        const auto test_pit_edges = get_pit_edges(elevation_map,test_pit_bbox,ELEVATION_THRESHOLD,test_pit_interior_points);
        auto way_points = generate_way_points(test_pit_edges,elevation_map,threshold_dist_from_pit,test_pit_interior_points);
//    //Pit interior point should ideally be an unordered_set. But making it a vector as of now.
//    // This is a design decision. a) There won't be lots of duplication b) The duplication doesn't harm us a lot
        cout<<endl<<"No. of pit interior points: "<<test_pit_interior_points.size()<<endl;
        cout<<endl<<"No. of Initial Waypoints : "<<way_points.size()<<endl;

        for(int i=0;i<elevation_map.size();i++)
        {
            for(int j=0;j<elevation_map[0].size();j++)
            {
                if(elevation_map[i][j]<=-79.0)
                {
                    test_pit_interior_points.emplace_back(coordinate{i,j});
                }
            }
        }

        auto true_way_points = get_true_waypoints(way_points,occupancy_map,test_pit_interior_points);
        cout<<endl<<"No. of final Waypoints : "<<true_way_points.size()<<endl;

//        unordered_set<coordinate,my_coordinate_hasher> pit_interior_set;
//        unordered_set<coordinate,my_coordinate_hasher> true_waypoints_set;
//
//        for(const auto &x:test_pit_interior_points)
//            pit_interior_set.insert(x);
//
//        for(const auto &x:true_way_points)
//            true_waypoints_set.insert(x);
//
//        for(int i=0; i<occupancy_map.size();i++)
//        {
//            for(int j=0;j<occupancy_map[0].size();j++)
//            {
//                if(pit_interior_set.count(coordinate{i,j}))
//                    cout<<"X";
//                else if(true_waypoints_set.count(coordinate{i,j}))
//                    cout<<"?";
//                else
//                    cout<<occupancy_map[i][j];
//            }
//            cout<<endl;
//        }
//
//        cout<<"==========================================================="<<endl;

        ///Writing data to CSV's
        const string waypoints_file_name = "data/waypoints.csv";
        const string pit_interior_file_name = "data/pit_interior.csv";
        convert_vector_to_csv(test_pit_interior_points,pit_interior_file_name);
        convert_vector_to_csv(true_way_points,waypoints_file_name);
    }
    return 0;
}