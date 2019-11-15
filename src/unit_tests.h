//
// Created by Harsh Sharma on 03/11/19.
//

#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <unordered_set>
#include "coordinate.h"
#include "b_box.h"
#include "global_map.h"
#include "planning_map.h"
#include "utility.h"
#include "rover_parameters.h"

void test_get_path_to_vantage_point();

void generate_sample_map()
{
    const int N_ROWS = 11;
    const int N_COLS = 20;
    const double MAX_ELEVATION = 1.5;
    const double MIN_ELEVATION= 0.2;
    global_map g = global_map(N_ROWS,N_COLS,MAX_ELEVATION,MIN_ELEVATION);
    g.g_map = vector<vector<double>> {N_ROWS,vector<double>(N_COLS,1)};
    g.g_map[3][7] = 0;
    g.g_map[3][8] = 0;
    g.g_map[3][9] = 0;
    g.g_map[4][7] = 0;
    g.g_map[4][8] = 0;
    g.g_map[4][9] = 0;
    g.g_map[4][10] = 0;
    g.g_map[5][5] = 0;
    g.g_map[5][6] = 0;
    g.g_map[5][7] = 0;
    g.g_map[5][8] = 0;
    g.g_map[5][9] = 0;
    g.g_map[5][10] = 0;
    g.g_map[5][11] = 0;
    g.g_map[6][5] = 0;
    g.g_map[6][6] = 0;
    g.g_map[6][7] = 0;
    g.g_map[6][8] = 0;
    g.g_map[6][9] = 0;
    g.g_map[6][10] = 0;
    g.g_map[6][11] = 0;
    g.g_map[6][12] = 0;
    g.g_map[7][5] = 0;
    g.g_map[7][6] = 0;
    g.g_map[7][7] = 0;
    g.g_map[7][8] = 0;
    g.g_map[7][9] = 0;
    g.g_map[7][10] = 0;
    g.g_map[7][11] = 0;
    g.g_map[7][12] = 0;
    g.g_map[8][6] = 0;
    g.g_map[8][7] = 0;
    g.g_map[8][8] = 0;
    g.g_map[8][9] = 0;
    g.g_map[8][10] = 0;
    g.g_map[8][11] = 0;
    g.g_map[9][7] = 0;
    g.g_map[9][8] = 0;
    g.g_map[9][9] = 0;
    g.g_map[9][10] = 0;

    vector<coordinate> pit_interior{coordinate{6,8},
                                    coordinate{6,9},
                                    coordinate{7,8},
                                    coordinate{7,9}};

    vector<coordinate> initial_waypoints{coordinate{6,7},
                                         coordinate{5,8},
                                         coordinate{5,9},
                                         coordinate{5,10},
                                         coordinate{6,10},
                                         coordinate{7,10},
                                         coordinate{8,7},
                                         coordinate{8,8},
                                         coordinate{8,9},
                                         coordinate{8,10},
                                         coordinate{7,7}};

    for(int i=0; i<g.g_map.size();i++)
    {
        for(int j=0;j<g.g_map[0].size();j++)
        {
            cout<<g.g_map[i][j]<<"\t";
        }
        cout<<endl;
    }
    cout<<"======================================================================================="<<endl;
    auto c = get_true_waypoints(initial_waypoints,g.g_map,pit_interior);
    g.display_unit_test_map(pit_interior,
                            initial_waypoints,
                            c);
}

//======================================================================================================================

void test_get_true_waypoints()
{
    generate_sample_map();
}

//======================================================================================================================