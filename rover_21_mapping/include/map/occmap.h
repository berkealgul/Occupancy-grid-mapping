/*
    Date 24.01.2021
    Author: Berke Algül
*/

#ifndef OCC_MAP_H
#define OCC_MAP_H  

#include <algorithm>
#include "map.h"

class OccupancyMap : public Map
{
public:

    // Log max and min values for stricting values
    int LOG_MAX  = 100;
    int LOG_MIN  = 0;

    // log odds updates for free and occupied cells
    // if there is occupied cell we increase the value, decrease otherwise
    // hence as value gets bigger we can say there is a obstacle
    int LOG_FREE;
    int LOG_OCC;

    // if grid value above this value, we consider this cell is occupied
    float OCCUPIED_TRESHOLD; 

    // hold occupied points for costmap
    std::vector<std::vector<int>> occupiedPoints;

    // recent scan and odom information for occupancy grid
    // must be up to date before calling update
    float x, y, state; // state
    float yaw, min_a, max_a, da, range_max; // scan
    std::vector<float> ranges; // range

    OccupancyMap(ros::NodeHandle&);

    void update();
    void fill_line(int, int, int, int);
    void putFree(int, int);
    void putOcc(int, int);
    void addOccupiedPoint(int, int);
    void removeOccupiedPoint(int, int);

    int findPoint(std::vector<int>&);
};

#endif