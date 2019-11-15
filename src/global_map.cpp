//
// Created by Harsh Sharma on 05/10/19.
//

#include "global_map.h"
#include <iostream>
#include <random>
#include <iomanip>
#include <string>

using namespace std;

//=====================================================================================================================

void global_map::generate_map()
{   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(minimum_elevation,maximum_elevation-1);
    for(size_t i=0; i<rows;i++)
    {
        for(size_t j=0;j<cols;j++)
        {
            g_map[i][j]  = distribution(generator);
        }
    }
    make_pit_in_map();
    //display_vector(g_map);        //See elevation map
}

//=====================================================================================================================

vector<pair<int,int>> global_map::get_pit_boundary_coordinates()
{
    //These are hard_coded values as of now to test the validation of the concept
    vector<pair<int,int>> pit_boundary{make_pair(2,9),
                                       make_pair(2,10),
                                       make_pair(2,11),
                                       make_pair(2,12),
                                       make_pair(2,13),
                                       make_pair(2,14),
                                       make_pair(6,9),
                                       make_pair(6,10),
                                       make_pair(6,11),
                                       make_pair(6,12),
                                       make_pair(3,8),
                                       make_pair(3,14),
                                       make_pair(4,7),
                                       make_pair(4,14),
                                       make_pair(5,8),
                                       make_pair(5,13),
                                       make_pair(2,14),
    };
    return std::move(pit_boundary);
}

//=====================================================================================================================

vector<coordinate> global_map::get_pit_interior_coordinates()
{
    //These are hard_coded values as of now to test the validation of the concept
    const vector<pair<int,int>> pit_interior_coordinates{make_pair(3,9),
                                                         make_pair(3,10),
                                                         make_pair(3,11),
                                                         make_pair(3,12),
                                                         make_pair(3,13),
                                                         make_pair(4,8),
                                                         make_pair(4,9),
                                                         make_pair(4,10),
                                                         make_pair(4,11),
                                                         make_pair(4,12),
                                                         make_pair(4,13),
                                                         make_pair(5,9),
                                                         make_pair(5,10),
                                                         make_pair(5,11),
                                                         make_pair(5,12)

    };

    vector<coordinate> pit_interior_coords;
    pit_interior_coords.reserve(pit_interior_coordinates.size());
    for(const auto &coord: pit_interior_coordinates)
    {
        pit_interior_coords.emplace_back(coordinate(coord.first,coord.second));
    }
    return std::move(pit_interior_coords);
}

//=====================================================================================================================

vector<pair<int,int>> global_map::get_pit_bbox_coordinates()
{
    //These are hard_coded values as of now to test the validation of the concept
    vector<pair<int,int>> pit_bbox_coordinates{make_pair(1,5),
                                               make_pair(1,16),
                                               make_pair(8,5),
                                               make_pair(8,16)

    };
    return std::move(pit_bbox_coordinates);
}

//=====================================================================================================================

void global_map::make_pit_in_map()
{
    const auto pit_boundary = get_pit_boundary_coordinates();
    const auto points_in_pit = get_pit_interior_coordinates();

    for(auto x:pit_boundary)
    {
        g_map[x.first][x.second] = maximum_elevation+5;
    }

    for(auto point_in_pit:points_in_pit)
    {
        g_map[point_in_pit.x][point_in_pit.y] = minimum_elevation-5;
    }
}

//=====================================================================================================================

template <class T>
void global_map::display_vector(const vector<vector<T>> &vec)
{
    for(int i=0; i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            cout<<setprecision(2)<<vec[i][j]<<"\t";
        }
        cout<<endl;
    }
}

//=====================================================================================================================

void global_map::display_final_map()
{
    const auto pit_bbox_coordinates = get_pit_bbox_coordinates();
    const auto pit_boundary = get_pit_boundary_coordinates();
    //const auto points_in_pit = get_pit_interior_coordinates();

    vector<vector<string>> display_map(rows,vector<string>(cols,"."));
    for(auto x:pit_boundary)
    {
        display_map[x.first][x.second] = "X";
    }

    for(auto x:pit_bbox_coordinates)
    {
        display_map[x.first][x.second] = "#";
    }

    if(!way_points.empty())
    {
        for(const auto &waypoint:way_points)
        {
            display_map[waypoint.x][waypoint.y] = "?";
        }
    }

    if(!path.empty())
    {
        for(size_t i=0;i<path.size();i++)
        {
            display_map[path[i].x][path[i].y] = std::to_string(i);
        }
    }

    display_vector(display_map);
}

//=====================================================================================================================

void global_map::display_final_map(const coordinate &start_coord,
                                   const coordinate &goal_coord)
{
    const auto pit_bbox_coordinates = get_pit_bbox_coordinates();
    const auto pit_boundary = get_pit_boundary_coordinates();
    //const auto points_in_pit = get_pit_interior_coordinates();

    vector<vector<string>> display_map(rows,vector<string>(cols,"."));
    for(auto x:pit_boundary)
    {
        display_map[x.first][x.second] = "X";
    }

    for(auto x:pit_bbox_coordinates)
    {
        display_map[x.first][x.second] = "#";
    }

    if(!way_points.empty())
    {
        for(const auto &waypoint:way_points)
        {
            display_map[waypoint.x][waypoint.y] = "?";
        }
    }

    if(!path.empty())
    {
        for(size_t i=0;i<path.size();i++)
        {
            display_map[path[i].x][path[i].y] = std::to_string(i);
        }
    }
    display_map[start_coord.x][start_coord.y] = "S";
    display_map[goal_coord.x][goal_coord.y] = "G";
    display_vector(display_map);
}

//=====================================================================================================================

double global_map::get_maximum_elevation()
{
    return maximum_elevation;
}

//=====================================================================================================================

double global_map::get_minimum_elevation()
{
    return minimum_elevation;
}

//=====================================================================================================================

void global_map::display_unit_test_map(const vector<coordinate> &pit_interior,
                           const vector<coordinate> &potential_pit_edge,
                           const vector<coordinate> &true_pit_edge)
{
    vector<vector<string>> display_map(rows,vector<string>(cols,"."));

    for(auto x:potential_pit_edge)
    {
        display_map[x.x][x.y] = "0";
    }

    for(auto x:pit_interior)
    {
        display_map[x.x][x.y] = "X";
    }


    if(!true_pit_edge.empty())
    {
        for(const auto &waypoint:true_pit_edge)
        {
            display_map[waypoint.x][waypoint.y] = "?";
        }
    }
    display_vector(display_map);
}

//=====================================================================================================================