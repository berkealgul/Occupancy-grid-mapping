/*
    Date 24.01.2021
    Author: Berke Algül
*/

#include "map/occmap.h"

OccupancyMap::OccupancyMap(ros::NodeHandle& nh) : Map(nh)
{
    nh.getParam("/RoverMapping/OccupancyMap/log_free", LOG_FREE);  
    nh.getParam("/RoverMapping/OccupancyMap/log_occ", LOG_OCC); 
    nh.getParam("/RoverMapping/OccupancyMap/occupied_treshold", OCCUPIED_TRESHOLD);

    // for debugging useful during parameter change
    ROS_INFO_STREAM("OCCUPANCY_MAP-------------");
    ROS_INFO_STREAM("LOG_MAX "<< LOG_MAX);
    ROS_INFO_STREAM("LOG_MIN "<< LOG_MIN);
    ROS_INFO_STREAM("LOG_FREE "<< LOG_FREE);
    ROS_INFO_STREAM("LOG_OCC "<< LOG_OCC);
    ROS_INFO_STREAM("OCCUPIED_TRESHOLD "<< OCCUPIED_TRESHOLD);
    ROS_INFO_STREAM("RES " << RES);
    ROS_INFO_STREAM("W " << W);
    ROS_INFO_STREAM("H " << H);
    ROS_INFO_STREAM("FIXED_FRAME " << FIXED_FRAME);
    ROS_INFO_STREAM("--------------------------");
}

/*
update occupancy grid usinginverse mesarument model
NOTE: this assumes odom and scan are up to date 
*/
void OccupancyMap::update()
{
    float a = min_a + yaw;
    int xi = discirtize(x) + ORIGIN_X;
    int yi = discirtize(y) + ORIGIN_Y;

    for(int i = 0, n = ranges.size(); i < n; i++)
    {
        // limit scan range in case of infinity
        float d = std::min(ranges[i], range_max); 
        float x_r = d*cos(a) + x;
        float y_r = d*sin(a) + y;
        
        int xi_r = discirtize(x_r) + ORIGIN_X;
        int yi_r = discirtize(y_r) + ORIGIN_Y;
        
        a = a + da;

        // trace empty rays from scan
        fill_line(xi, yi, xi_r, yi_r); 

        // if scan range did not exceed maximum range, we detected an obstackle
        if(d < range_max)
        {
            putOcc(xi_r, yi_r);
        }
        else
        {
            putFree(xi_r, yi_r);
        }
    }
}

/*
update grid values using bresenham line tracing algorithm
updates only empty values. 
Note: uses grid coordinates

input:
    x1, y1 -> center coordinate of line
    x2, y2 -> end coordinate of line
*/
void OccupancyMap::fill_line(int x1, int y1, int x2, int y2)
{
    int dx = x2-x1;
    int dy = y2-y1;

    int sx = (dx>=0) ? 1 : (-1);
    int sy = (dy>=0) ? 1 : (-1);

    int x = x1;
    int y = y1;

    int isSwaped = 0;

    // swap if M > 1 
    if(abs(dy) > abs(dx))
    {
        int tdx = dx;
        dx =dy;
        dy = tdx;

        isSwaped = 1;
    }

    int p = 2*(abs(dy)) - abs(dx);

    //Print Initial Point
    putFree(x,y);
    
    // Loop for dx times
    for(int i = 0, n = abs(dx)-1; i< n; i++)
    {
        // Depending on decision parameter
        if(p < 0)
        {
            if(isSwaped == 0)
            {
                x = x + sx;
            }
            else
            {
                y = y+sy;             
            }
            p = p + 2*abs(dy);
        }
        else
        {
            x = x+sx;
            y = y+sy;
            p = p + 2*abs(dy) - 2*abs(dx);
        }

        putFree(x,y);

        // went outside break loop
        // MAYBE UNECESSARY
        if(!inRange(x,y))
        {
            return;
        }
    }
}

/*
Put free value to grid coordinate

input:
    x, y -> obstackle free coordinate
*/
void OccupancyMap::putFree(int x, int y)
{
    if(!inRange(x, y))
    {
        return;
    }
    
    int idx = I(x, y);
    int val = std::max(grid.data[idx] + LOG_FREE, LOG_MIN);
    grid.data[idx] = val;
    
    if(val < OCCUPIED_TRESHOLD)
    {
        removeOccupiedPoint(x, y);
    }

}

/*
Put occupied value to grid coordinate

input:
    x, y -> obstackle coordinate
*/
void OccupancyMap::putOcc(int x, int y)
{
    if(!inRange(x, y))
    {
        return;
    }

    int idx = I(x, y);
    int val = std::min(grid.data[idx] + LOG_OCC, LOG_MAX);
    grid.data[idx] = val;
    
    if(val >= OCCUPIED_TRESHOLD)
    {
        addOccupiedPoint(x, y);
    }
}

/*
adds occupied x,y coordinate

input:
    x, y -> occupied coordinate
*/
void OccupancyMap::addOccupiedPoint(int x, int y)
{
    std::vector<int> point;
    point.push_back(x);
    point.push_back(y);

    int pos = findPoint(point);

    if (pos == -1)
    {
        occupiedPoints.push_back(point);
    }
}

/*
removes occupied x,y coordinate

input:
    x, y -> occupied coordinate
*/
void OccupancyMap::removeOccupiedPoint(int x ,int y)
{
    std::vector<int> point;
    point.push_back(x);
    point.push_back(y);

    int pos = findPoint(point);

    if (pos != -1)
    {
        occupiedPoints.erase(occupiedPoints.begin()+pos);
    }
}

/*
tries to finds coordinate in occupied coordinates

input:
    vector -> coordinate tuple that needs to be found

return:
    int -> index of the point returns -1 if coordinate is not found
*/
int OccupancyMap::findPoint(std::vector<int> &p)
{
    for(int i = 0, n = occupiedPoints.size(); i < n; i++)
    {
        if(p == occupiedPoints[i])
        {
            return i;
        }
    }

    return -1;
}
