//
// Created by Harsh Sharma on 06/10/19.
//

#pragma once

#include "coordinate.h"

//=====================================================================================================================

struct Node
{
    coordinate c;
    coordinate parent;
    double gcost;
    double hcost;
    double fcost;
    static int map_width;

    Node(coordinate c1,
         coordinate c_parent,
         double g_cost,
         double h_cost):
         c(c1),parent(c_parent),gcost(g_cost),hcost(h_cost){
         fcost = calculate_fcost();
    }

    Node(coordinate c1):
    c{c1}, parent{-1,-1}, gcost{INT_MAX}, hcost{INT_MAX}{
        fcost = calculate_fcost();
    }

    double calculate_fcost();
    double calculate_hcost(const coordinate &goal_coordinate) const;
    void set_fcost(const double &new_f_cost);
    void set_hcost(const double &new_h_cost);
    void set_gcost(const double &new_g_cost);
    void set_parent(const coordinate &new_parent);
    void print_node() const;
    static void set_map_width(const int &m_width) { map_width = m_width;}

};

//=====================================================================================================================

inline bool operator < (const Node& lhs, const Node& rhs)
{
    return lhs.fcost < rhs.fcost;
}

//=====================================================================================================================

inline bool operator == (const Node& lhs, const Node& rhs)
{
    return lhs.c == rhs.c;
}

//=====================================================================================================================

inline bool operator != (const Node& lhs, const Node& rhs)
{
    return !(lhs==rhs);
}

//=====================================================================================================================

struct Comp{
    bool operator()(const Node &a, const Node &b){
        return a.fcost>b.fcost;
    }
};

//=====================================================================================================================

struct node_hasher
{
    size_t operator()(const Node &obj) const;
};

//=====================================================================================================================