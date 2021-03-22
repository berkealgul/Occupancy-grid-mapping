/*
    Date 24.01.2021
    Author: Berke Algül
*/

#ifndef MAP_H
#define MAP_H  

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class Map
{
public:
    nav_msgs::OccupancyGrid grid;

    // w (m) = W * RES    h (m) = H * RES
    float RES; // resulution of the grid (per meter for each cell)
    int W, H; // width and height of the grid (gridwise lenght)
    // NOTE: for calculating map width and lenght as meters

    // origin coordinates (gridwise) of rover in grid
    int ORIGIN_X;
    int ORIGIN_Y;

    std::string FIXED_FRAME;

    // this is initial values of grid. meant to be use when cleaning the grid
    std::vector<int8_t> defaultGridVals;

    Map(ros::NodeHandle&);

    void update();
    void initalize_grid();
    void clear();

    int discirtize(float);
    int I(int, int);

    bool inRange(int, int);

    nav_msgs::OccupancyGrid msg();
};

#endif