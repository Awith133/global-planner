//
// Created by Harsh Sharma on 21/10/19.
//

#pragma once

struct rover_parameters
{
    double velocity;    //in m/s

    explicit rover_parameters(double vel = 0.05): velocity{vel} {}
};
