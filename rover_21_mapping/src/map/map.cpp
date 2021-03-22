/*
    Date 24.01.2021
    Author: Berke Algül
*/

#include "map/map.h"

// we need nodehandle to load params
Map::Map(ros::NodeHandle& nh)
{
    nh.getParam("/RoverMapping/width", W);
    nh.getParam("/RoverMapping/height", H);
    nh.getParam("/RoverMapping/resolution", RES);
    nh.getParam("/RoverMapping/fixed_frame", FIXED_FRAME);

    // set origin to center of the grid
    ORIGIN_X = W/2;
    ORIGIN_Y = H/2;

    initalize_grid();
}

/*
Sets grid values to initial values
*/
void Map::clear()
{
    grid.data = defaultGridVals;
}

/*
convert continious world 2d point to discrite grid coordinates
NOTE: origin coordinates are not added
input:
    x -> continious 2d coordinate
output
    discritized grid coordinate (x or y axis and origin offset is not added)
*/
int Map::discirtize(float x)
{
    return ceil(x/RES);
}

/*
convert x, y (column, row) = (x, y) to 1d index for ros  occupacy grid message)
NOTE: according to ros, occupancy grid is row-major order 1d list

input:
    x, y -> 2d grid coordinate

returns
    int -> 1d array index in range [0, W*H)
*/
int Map::I(int x, int y)
{
    return y * W + x;
}

/*

Checks if given coordinate (grid coordinates)
is in inside of grid

inputs:
    x, y -> grid coordinates
returns
    bool -> true if coordinate is inside of the grid

*/
bool Map::inRange(int x, int y)
{
    return x >= 0 && x < W && y >= 0 && y < H;
}

/*
    Fills grid msg to its initial state
*/
void Map::initalize_grid()
{
    grid.header.frame_id = FIXED_FRAME; // set map frame
    grid.info.resolution = RES;
    grid.info.width = W;
    grid.info.height = H;

    // origin points
    grid.info.origin.position.x = -RES*W/2;
    grid.info.origin.position.y = -RES*H/2;

    // fill with values
    for(int i = 0, n = W*H; i < n; i++)
    {
        grid.data.push_back(-1);
    }

    defaultGridVals = grid.data;
}

/*
    updates its time stamp and returns msg to be published
*/
nav_msgs::OccupancyGrid Map::msg()
{
    grid.header.stamp = ros::Time::now();
    return grid;
}
