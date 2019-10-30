//
// Created by Harsh Sharma on 21/10/19.
//

#include "mga_node.h"
#include "map_width_header.h"

//=====================================================================================================================

void MGA_Node::print_MGA_node() const {
    n.print_node();
    cout<<"Time taken to reach coordinate: "<<time_to_reach<<endl;
}

//=====================================================================================================================

size_t MGA_node_hasher::operator()(const MGA_Node &obj) const
{
    return obj.n.c.x * GLOBAL_MAP_WIDTH + obj.n.c.y;
}

//=====================================================================================================================