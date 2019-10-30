//
// Created by Harsh Sharma on 05/10/19.
//

#pragma once

#include <iostream>

using namespace std;

struct coordinate
{
    int x;
    int y;
    coordinate(int x_coord,
               int y_coord):
            x(x_coord),y(y_coord){}

    void print_coordinate() const
    {
        cout<<"x: "<<x<<"\t"<<"y: "<<y<<endl;
    }

    double get_euclidian_distance(const coordinate &c) const;

    friend bool operator== (const coordinate &lhs, const coordinate &rhs);
    friend bool operator!= (const coordinate &lhs, const coordinate &rhs);
};

//=====================================================================================================================

struct my_coordinate_hasher
{
    size_t operator()(const coordinate &obj) const;
};

//=====================================================================================================================
