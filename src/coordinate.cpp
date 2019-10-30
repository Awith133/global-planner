//
// Created by Harsh Sharma on 05/10/19.
//

#include "coordinate.h"
#include <cmath>
#include "map_width_header.h"

bool operator==(const coordinate& lhs, const coordinate& rhs)
{
    return ((lhs.x==rhs.x)&&(lhs.y==rhs.y));
}

bool operator!=(const coordinate& lhs, const coordinate& rhs)
{
    return !(lhs==rhs);
}

//=====================================================================================================================

double coordinate::get_euclidian_distance(const coordinate &c) const
{
    return (sqrt((this->x - c.x)*(this->x - c.x)
                 + (this->y - c.y)*(this->y - c.y)));
}

//=====================================================================================================================

size_t my_coordinate_hasher::operator()(const coordinate &obj) const
{
    return obj.x * GLOBAL_MAP_WIDTH + obj.y;
}

//=====================================================================================================================

