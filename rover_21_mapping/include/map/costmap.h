/*
    Date 24.01.2021
    Author: Berke Algül
*/

#ifndef COST_MAP_H
#define COST_MAP_H  

#include "map.h"

class Costmap : public Map
{
public:
    // in meters
    float LETHAL_COST_R;
    float TOTAL_COST_R;

    // in grid cell length
    int lethalCostGridR;
    int totalCostGridR;

    int MAX_COST = 100;
    int MIN_COST = 1;

    int costDecay;

    Costmap(ros::NodeHandle&);

    void update(std::vector<std::vector<int>>);
    void inflate(int, int);
    void fill_circle(int, int, int, int);
    void fill_circle_midpoint(int, int, int, int);
    void fill(int, int, int, int, int);
    void putCost(int, int, int);
};

#endif