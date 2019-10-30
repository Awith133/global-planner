//
// Created by Harsh Sharma on 05/10/19.
//

#pragma once

using namespace std;

//=====================================================================================================================

struct bbox
{
    int x_min;
    int y_min;
    int x_max;
    int y_max;
    bbox(int x_min_coord,
         int y_min_coord,
         int x_max_coord,
         int y_max_coord):
            x_min(x_min_coord),y_min(y_min_coord),x_max(x_max_coord),y_max(y_max_coord){}

    void get_bbox_coord(const vector<pair<int,int>> &pit_bbox)
    {
        int X_MIN = INT_MAX;
        int Y_MIN = INT_MAX;
        int X_MAX = INT_MIN;
        int Y_MAX = INT_MIN;
        for(const auto elt:pit_bbox)
        {
            auto x_coord = elt.first;
            auto y_coord = elt.second;
            if(x_coord>X_MAX)
                X_MAX = x_coord;
            else if (x_coord<X_MIN)
                X_MIN = x_coord;

            if(y_coord>Y_MAX)
                Y_MAX = y_coord;
            else if (y_coord<Y_MIN)
                Y_MIN = y_coord;
        }
        x_min = X_MIN;
        y_min = Y_MIN;
        x_max = X_MAX;
        y_max = Y_MAX;
    }

    int get_mid_x()
    {
        //Mid can be double- but I just round it to integer
        return 0.5*(x_min + x_max);
    }

    int get_mid_y()
    {
        //Mid can be double- but I just round it to integer
        return 0.5*(y_min + y_max);
    }
};

//=====================================================================================================================