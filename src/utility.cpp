//
// Created by Harsh Sharma on 06/10/19.
//

#include "utility.h"
#include "planning_map.h"
#include "node.h"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include "mga_node.h"
#include "map_width_header.h"
#include "b_box.h"

int GLOBAL_MAP_WIDTH;

//=====================================================================================================================

bool is_destination(const coordinate &c, const coordinate &goal)
{
    return c == goal;

}

//=====================================================================================================================

vector<coordinate> get_neighbors(const int &x,
                                 const int &y,
                                 const vector<vector<double>> &map)
{
    constexpr int NUMOFDIRS = 4; //Assume 4 connected grid for now
    int dX[NUMOFDIRS] = {-1, 1, 0, 0};
    int dY[NUMOFDIRS] = {0, 0, -1, 1};
    vector<coordinate> neighbors;
    for(int dir = 0; dir < NUMOFDIRS; dir++) {
        int newx = x + dX[dir];
        int newy = y + dY[dir];
        if (newx >= 0 && newx < map.size() && newy >= 0 && newy < map[0].size())
        {
            neighbors.emplace_back(coordinate(newx,newy));
        }
    }
    return std::move(neighbors);
}

//=====================================================================================================================

bool is_coordinate_pit_edge(const int &x,
                            const int &y,
                            const vector<vector<double>> &map,
                            const vector<coordinate> &neighbors,
                            const double &threshold,
                            vector<coordinate> &pit_interior)
{
    /// Note: This function also changes the pit_interior. If P is greater than Q by more than the threshold elevation
    /// then P is marked as Pit edge and Q is marked as Pit interior
    /// Verify the interior aspect of this function.
    int flag=0;
    for(const auto &neighbor:neighbors)
    {
        if(abs(abs(map[x][y]) - abs(map[neighbor.x][neighbor.y]))>threshold)
//        if(map[x][y] - map[neighbor.x][neighbor.y]>threshold)
        {
            flag=1;
            pit_interior.emplace_back(coordinate(x,y));
        }
    }
    return flag;
}

//=====================================================================================================================

vector<coordinate> get_pit_edges(const vector<vector<double>> &map,
                                 const vector<pair<int,int>> &pit_bbox,
                                 const double &threshold,
                                 vector<coordinate> &pit_interior_points)
{
//  The threshold as of now is based on the difference of the max and min elevation

    bbox b(0,0,0,0);
    b.get_bbox_coord(pit_bbox);
//    cout<<b.x_min<<"\t"<<b.y_min<<"\t"<<b.x_max<<"\t"<<b.y_max<<endl;
    vector<coordinate> pit_edges;
    for(size_t i=b.x_min;i<=b.x_max;i++)
    {
        for(size_t j=b.y_min;j<=b.y_max;j++)
        {   //Note: Since we know that pit_bbox wont be a very large 2D vector O(n^3) is fine. See if it can be optimised
            const auto neighbors = get_neighbors(i,j,map);
            if(is_coordinate_pit_edge(i,j,map,neighbors,threshold,pit_interior_points))
                pit_edges.emplace_back(coordinate(i,j));
        }
    }
    return pit_edges;
}

//=====================================================================================================================

void expand_state(const Node &node_to_expand,
                  priority_queue<Node, vector<Node>, Comp> &open,
                  vector<vector<Node>> &node_map,
                  const vector<int> &dX,
                  const vector<int> &dY,
                  const planning_map &elevation_map,
                  unordered_set<Node,node_hasher> &closed)
{
    const auto current_x = node_to_expand.c.x;
    const auto current_y = node_to_expand.c.y;
    const int cost_per_step = 1;
    for(size_t dir = 0; dir < dX.size(); dir++)
    {
        int newx = current_x + dX[dir];
        int newy = current_y + dY[dir];
        coordinate new_coordinate {newx,newy};

        if (elevation_map.is_valid(new_coordinate))
        {
            if(!closed.count(node_map[newx][newy]) && (node_map[newx][newy].gcost > node_map[current_x][current_y].gcost + cost_per_step))
            {
                node_map[newx][newy].set_gcost(node_map[current_x][current_y].gcost + cost_per_step);
                node_map[newx][newy].set_parent(node_to_expand.c);
                open.push(node_map[newx][newy]);
            }
        }
    }
}

//=====================================================================================================================

vector<coordinate> backtrack(  const Node &start_node,
                                const Node &goal_node,
                                vector<vector<Node>> &node_map)
{
    vector<coordinate> path;
    Node curr_node = goal_node;
    while(curr_node!=start_node)
    {
        path.emplace_back(curr_node.c);
        curr_node = node_map[curr_node.parent.x][curr_node.parent.y];
    }
    std::reverse(path.begin(),path.end());
    return std::move(path);
}

//=====================================================================================================================

vector<coordinate> astar(const coordinate &start,const coordinate &goal,const planning_map &elevation_map){

    vector<coordinate> path{};
    if (!elevation_map.is_valid(start)) {
        std::cout << "Start is an obstacle" << std::endl;
        return path;
        //Start is invalid
    }

    if (!elevation_map.is_valid(goal)) {
        std::cout << "Destination is an obstacle" << std::endl;
        return path;
        //Destination is invalid
    }

    if (is_destination(start, goal)) {
        std::cout << "Already at the destination" << std::endl;
        return path;
    }
    GLOBAL_MAP_WIDTH =  elevation_map.map[0].size();
    const vector<int> dX = {-1, -1, -1,  0,  0,  1, 1, 1};
    const vector<int> dY {-1,  0,  1, -1,  1, -1, 0, 1};

    const Node random_init_node{coordinate{-1,-1},coordinate{-1,-1},INT_MAX,INT_MAX};
    vector<vector<Node>> node_map(elevation_map.map.size(),vector<Node> (elevation_map.map[0].size(),random_init_node));
    for(int i=0;i<elevation_map.map.size();i++)
    {
        for(int j=0;j<elevation_map.map[0].size();j++)
        {
            node_map[i][j].c = coordinate{i,j};
            node_map[i][j].set_hcost(node_map[i][j].calculate_hcost(goal));
        }
    }

    priority_queue<Node, vector<Node>, Comp> open;
    unordered_set<Node,node_hasher> closed;

    Node start_node(start,coordinate{-1,-1},0,INT_MAX);
    start_node.set_hcost(start_node.calculate_hcost(goal));
    Node goal_node(goal,coordinate{-1,-1},INT_MAX,0);
    node_map[start_node.c.x][start_node.c.y] = start_node;
    node_map[goal_node.c.x][goal_node.c.y] = goal_node;

    open.push(start_node);
    while (!open.empty() && !closed.count(goal_node))
    {
        const auto node_to_expand = open.top();
        open.pop();
        if(closed.count(node_to_expand)==0)       //Added this new condition to avoid multiple expansion of the same state
        {
            closed.insert(node_to_expand);
            expand_state(node_to_expand,open,node_map,dX,dY,elevation_map,closed);
        }
    }
    if(open.empty())
    {
        cout<<"No path to goal"<<endl;
    }
    else
    {
        cout<<"Path to goal found!"<<endl;
        path = backtrack(node_map[start_node.c.x][start_node.c.y],node_map[goal_node.c.x][goal_node.c.y],node_map);
    }
    return std::move(path);
}

//=====================================================================================================================

double get_MGA_heuristic(const coordinate &start_coordinate,
                         const vector<coordinate> &goals)
{
    /// This heuristic is Euclidian as of now.
    double min_heuristic = INT_MAX;
    for(const auto &goal:goals)
    {
        min_heuristic = std::min(min_heuristic,start_coordinate.get_euclidian_distance(goal));
    }
    return min_heuristic;
}

//=====================================================================================================================

void implicit_expand_state(const MGA_Node  &node_to_expand,
                          priority_queue<MGA_Node, vector<MGA_Node>, MGA_Comp> &open,
                          const vector<int> &dX,
                          const vector<int> &dY,
                          const planning_map &elevation_map,
                          unordered_set<MGA_Node,MGA_node_hasher> &closed,
                          unordered_set<MGA_Node,MGA_node_hasher> &node_map,
                           const vector<coordinate> &goals,
                          const rover_parameters &rover_config)
{
    const auto current_x = node_to_expand.n.c.x;
    const auto current_y = node_to_expand.n.c.y;
    const double cost_per_step = 1;                    //See if you want to change the diagonal traversal value
    const double pixel_distance = 5;                //This is the per pixel distance/discretization
    const double time_of_traversal_per_pixel= pixel_distance/rover_config.velocity;

    for(size_t dir = 0; dir < dX.size(); dir++)
    {
        int newx = current_x + dX[dir];
        int newy = current_y + dY[dir];
        coordinate new_coordinate {newx,newy};
        const MGA_Node temp_mga_node{new_coordinate};

        if (elevation_map.is_valid(new_coordinate) && !closed.count(temp_mga_node))
        {

            double present_g_cost = INT_MAX;
            double transition_cost = cost_per_step;
            double traversal_time = time_of_traversal_per_pixel;

            auto node_in_consideration = node_map.find (temp_mga_node);
            if(node_in_consideration!=node_map.end())
            {
                present_g_cost = node_in_consideration->n.gcost;
            }

            if(abs(dX[dir]) && abs(dY[dir]))
            {
                traversal_time = 1.4*time_of_traversal_per_pixel;
                transition_cost =  1.4*cost_per_step;
            }

            if(present_g_cost > node_to_expand.n.gcost + transition_cost)
            {
                if(node_in_consideration!=node_map.end())
                {
                    node_map.erase(node_in_consideration);      //Node map is an unordered set. An existing element has to be deleted, it can't be changed.
                }

                double new_gcost = node_to_expand.n.gcost + transition_cost;
                double new_hcost = get_MGA_heuristic(new_coordinate,goals);
                MGA_Node new_MGA_node{new_coordinate,node_to_expand.n.c,new_gcost,new_hcost,node_to_expand.time_to_reach+traversal_time};
                node_map.insert(new_MGA_node);
                //Validate if gcost,hcost and fcost are getting updated
                open.push(new_MGA_node);

            }
        }
    }
}

//=====================================================================================================================

MGA_Node get_best_goal(unordered_map<MGA_Node,double,MGA_node_hasher> &goal_traversal_times,
                       const multi_goal_A_star_configuration &MGA_config,
                       const vector<double> &time_remaining_to_lose_vantage_point_status,
                       bool &vantage_point_reached_within_time,
                       const vector<coordinate> &goals)
{
    double best_time_stat = INT_MIN;
    MGA_Node best_goal{coordinate{-1,-1}};

    for(size_t i=0;i<time_remaining_to_lose_vantage_point_status.size();i++)
    {
        MGA_Node temp_mga_node{goals[i]};
        auto node_in_consideration = goal_traversal_times.find (temp_mga_node);
//        cout<<"Time taken to reach: "<<node_in_consideration->second<<endl;
//        cout<<"time_remaining_to_lose_vantage_point_status: "<< time_remaining_to_lose_vantage_point_status[i]<<endl;
//        cout<<"Difference inclusive of pessimistic factor "<<time_remaining_to_lose_vantage_point_status[i] - MGA_config.pessimistic_factor*node_in_consideration->second<<endl;
//        node_in_consideration->first.print_MGA_node();
//        cout<<"============================================================="<<endl;
        if(time_remaining_to_lose_vantage_point_status[i] - MGA_config.pessimistic_factor*node_in_consideration->second > best_time_stat)
        {
            best_time_stat = time_remaining_to_lose_vantage_point_status[i] - MGA_config.pessimistic_factor*node_in_consideration->second;
            best_goal = node_in_consideration->first;
        }
    }

    if(best_time_stat>0)
        vantage_point_reached_within_time=true;

    cout<<"Best time stat is: "<<best_time_stat<<endl;
//    best_goal.print_MGA_node();
    return best_goal;
}

//=====================================================================================================================

vector<coordinate> MGA_backtrack(MGA_Node start_node,
                                 MGA_Node goal_node,
                                 const unordered_set<MGA_Node,MGA_node_hasher> &node_map)
{
    vector<coordinate> path;
    MGA_Node curr_node = goal_node;
    while(curr_node!=start_node)
    {
        path.emplace_back(curr_node.n.c);
        MGA_Node temp_node{curr_node.n.parent};
        auto it = node_map.find(temp_node);
        curr_node = *it;
    }
//    path.emplace_back(start_node.n.c);
    std::reverse(path.begin(),path.end());
    return std::move(path);
}

//=====================================================================================================================

multi_goal_A_star_return multi_goal_astar(const coordinate &start,
                                  const vector<coordinate> &goals,
                                  const planning_map &elevation_map,
                                  const vector<double> &time_remaining_to_lose_vantage_point_status,
                                  const rover_parameters &rover_config,
                                  const multi_goal_A_star_configuration &MGA_config)
{
    ///Handle case where start point is possible goal point as well. Worst case prune it from the list of possible goals.
    const vector<int> dX = {-1, -1, -1,  0,  0,  1, 1, 1};
    const vector<int> dY {-1,  0,  1, -1,  1, -1, 0, 1};

    priority_queue<MGA_Node, vector<MGA_Node>, MGA_Comp> open;
    unordered_set<MGA_Node,MGA_node_hasher> closed;
    unordered_set<MGA_Node,MGA_node_hasher> goals_set;
    unordered_set<MGA_Node,MGA_node_hasher> node_map;   //This serves as my map since it's an implicit graph
    unordered_map<MGA_Node,double,MGA_node_hasher> goal_traversal_times;

    for(const auto &goal:goals)
    {
        goals_set.insert(MGA_Node{goal});
    }
    MGA_Node start_node(start,coordinate{-1,-1},0,INT_MAX,0);
    start_node.n.set_hcost(get_MGA_heuristic(start,goals));
    open.push(start_node);
    node_map.insert(start_node);

    int goals_expanded = 0;
    while (!open.empty() && goals_expanded!=goals.size())
    {
        const auto node_to_expand = open.top();

        if(goals_set.count(node_to_expand) && !closed.count(node_to_expand))
        {
            goals_expanded++;
            goal_traversal_times[node_to_expand] = node_to_expand.time_to_reach;
//            cout<<"Goal Reached "<<endl;
//            node_to_expand.n.c.print_coordinate();
//            cout<<goal_traversal_times.size()<<"============================"<<endl;
        }

        open.pop();
        if(closed.count(node_to_expand)==0)       //Added this new condition to avoid multiple expansion of the same state
        {
            closed.insert(node_to_expand);
            implicit_expand_state(node_to_expand,open,dX,dY,elevation_map,closed,node_map,goals,rover_config);
        }
    }
    bool vantage_point_reached_within_time = false;
    auto best_goal = get_best_goal(goal_traversal_times,MGA_config,time_remaining_to_lose_vantage_point_status,vantage_point_reached_within_time,goals);
    if(!vantage_point_reached_within_time)
        return multi_goal_A_star_return{-1,vector<coordinate> {}};

    const auto pessimistic_time_estimate_to_reach_best_goal = MGA_config.pessimistic_factor*goal_traversal_times[best_goal];
    auto path = MGA_backtrack(start_node,best_goal,node_map);
    cout<<"Path size is "<<path.size()<<endl;
    cout<<"Time taken to reach waypoint "<<pessimistic_time_estimate_to_reach_best_goal<<endl;
    return multi_goal_A_star_return{pessimistic_time_estimate_to_reach_best_goal,path};
}

//=====================================================================================================================

multi_goal_A_star_return get_path_to_vantage_point(const vector<vector<double>> &g_map,
                                             const double &min_elevation,
                                             const double &max_elevation,
                                             const coordinate &start_coordinate,
                                             const vector<coordinate> &goal_coordinates,
                                             const vector<double> &time_remaining_to_lose_vantage_point_status,
                                             const rover_parameters &rover_config)
{
    planning_map my_map{g_map,min_elevation,max_elevation}; //Pit interiors have to be made obstacle here. Tune min elevation according to that
    const multi_goal_A_star_configuration MGA_config{2,5};
    return multi_goal_astar(start_coordinate,goal_coordinates,my_map,time_remaining_to_lose_vantage_point_status,rover_config,MGA_config);
}

//=====================================================================================================================


vector<coordinate> get_goal_coordinates(const vector<vector<double>> &lit_waypoint_time_data,
                                        const int &present_time_index,
                                        const vector<coordinate> &original_waypoints,
                                        const unordered_set<coordinate,my_coordinate_hasher> &visited_waypoints,
                                        const double &time_per_step,
                                        vector<double> &time_remaining_to_lose_vantage_point_status)
{
    vector<coordinate> goal_coordinates;
    /// TO DO: See what Sohil gives you in the lit_waypoint_time_data array. Based on that you would have to complete this code.
    /// See if it is index based on the original distribution of waypoints list? Or does the table have in the first column, the waypoints coordinate
    for(size_t i=0;i<lit_waypoint_time_data.size();i++)
    {
        if(lit_waypoint_time_data[i][present_time_index]>0 && !visited_waypoints.count(original_waypoints[i]))
        {
            goal_coordinates.emplace_back(original_waypoints[i]);
            time_remaining_to_lose_vantage_point_status.emplace_back(lit_waypoint_time_data[i][present_time_index]*time_per_step);
        }
    }
    return std::move(goal_coordinates);
}

//=====================================================================================================================

std::vector<double> split(const std::string& s, char delimiter)
{
    std::vector<double> result;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        result.push_back(stod(token));
    }
    return result;
}

//=====================================================================================================================

vector<vector<double>> convert_csv_to_vector(const string &file_name)
{
    std::ifstream file(file_name);
    string line;
    string number;
    string temp;
    vector<vector<double>> map;
    int count=0;

    while (getline(file, line,'\n'))
    {
        auto res = split(line,',');
        map.push_back(res);
    }

    cout<<"Map_Rows: "<<map.size()<<endl;
    cout<<"Map_Col: "<<map[0].size()<<endl;
//    //testing
//    for(size_t i=0;i<map.size();i++)
//    {
//        for(size_t j=0;j<map[0].size();j++)
//        {
//            cout<<map[i][j]<<"\t";
//        }
//        cout<<endl;
//    }

    return std::move(map);
}

//=====================================================================================================================

void convert_vector_to_csv(const vector<coordinate> &vec,const string &file_name)
{
    ofstream myfile(file_name);
    int vsize = vec.size();
    for (int n=0; n<vsize; n++)
    {
        myfile << vec[n].x <<","<< vec[n].y << endl;
    }
    cout<<"CSV file created"<<endl;
}

//=====================================================================================================================

vector<coordinate> make_coordinate_vector_from_csv(const string &file_name)
{
    std::ifstream file(file_name);
    string line;
    string number;
    string temp;
    vector<coordinate> coord_vector;

    while (getline(file, line,'\n'))
    {
        auto res = split(line,',');
        coord_vector.emplace_back(coordinate{static_cast<int>(res[0]),static_cast<int>(res[1])});
    }

    //testing
//    for(size_t i=0;i<coord_vector.size();i++)
//    {
//        cout<<i<<"\t";
//        coord_vector[i].print_coordinate();
//    }

    return std::move(coord_vector);
}

//=====================================================================================================================

//template <typename T>
void convert_tuple_vector_to_csv(vector<tuple<int,int,int>> time_location, const string &file_name)
{
    ofstream myfile(file_name);
    for (int n=0; n<time_location.size(); n++)
    {
        myfile << get<0>(time_location[n])<<","<<get<1>(time_location[n])<<","<<get<2>(time_location[n]) << endl;
    }
    cout<<"CSV file "<<file_name<<"created"<<endl;
}

//=====================================================================================================================