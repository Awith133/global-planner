//
// Created by Harsh Sharma on 21/10/19.
//

#pragma once

#include "node.h"

//=====================================================================================================================

struct MGA_Node
{
    Node n;
    double time_to_reach;

    MGA_Node(coordinate c1,
                  coordinate c_parent,
                  double g_cost,
                  double h_cost,
                  double time):
            n{c1,c_parent,g_cost,h_cost},time_to_reach{time}{
    }

    explicit MGA_Node(coordinate c1):
            n{c1},time_to_reach{INT_MAX}{
    }

    MGA_Node(Node new_node,double time):
            n(new_node),time_to_reach(time){}

    void print_MGA_node() const;
};

//=====================================================================================================================

inline bool operator < (const MGA_Node& lhs, const MGA_Node& rhs)
{
    return lhs.n.fcost < rhs.n.fcost;
}

//=====================================================================================================================

inline bool operator == (const MGA_Node& lhs, const MGA_Node& rhs)
{
    return lhs.n.c == rhs.n.c;
}

//=====================================================================================================================

inline bool operator != (const MGA_Node& lhs, const MGA_Node& rhs)
{
    return !(lhs==rhs);
}

//=====================================================================================================================

struct MGA_Comp{
    bool operator()(const MGA_Node &a, const MGA_Node &b){
        return a.n.fcost>b.n.fcost;
    }
};

//=====================================================================================================================

struct MGA_node_hasher
{
    size_t operator()(const MGA_Node &obj) const;
};

//=====================================================================================================================
