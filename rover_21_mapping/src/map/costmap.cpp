/*
    Date 24.01.2021
    Author: Berke Algül
*/

#include "map/costmap.h"

Costmap::Costmap(ros::NodeHandle& nh) : Map(nh)
{
    nh.getParam("/RoverMapping/Costmap/total_cost_r", TOTAL_COST_R);
    nh.getParam("/RoverMapping/Costmap/lethal_cost_r", LETHAL_COST_R);
    nh.getParam("/RoverMapping/Costmap/max_cost", MAX_COST);
    nh.getParam("/RoverMapping/Costmap/min_cost", MIN_COST);

    lethalCostGridR = discirtize(LETHAL_COST_R);
    totalCostGridR = discirtize(TOTAL_COST_R);

    costDecay = MAX_COST / lethalCostGridR;

    // ideal degil ama is yapiyor
    // TODO: haritayı yeniden dolrdurmak yerine
    // ilk seferde nasıl doldurabileceğine bak 
    // initialize_grid in üzerine yazamadın niye?
    for(int i = 0, n = W*H; i < n; i++)
    {
        grid.data[i] = MIN_COST;
        defaultGridVals[i] = MIN_COST;
    }

    // for debugging useful during parameter change
    ROS_INFO_STREAM("COSTMAP-------------------");
    ROS_INFO_STREAM("LETHAL_COST_R "<< LETHAL_COST_R);
    ROS_INFO_STREAM("TOTAL_COST_R "<< TOTAL_COST_R);
    ROS_INFO_STREAM("MAX_COST "<< MAX_COST);
    ROS_INFO_STREAM("MIN_COST "<< MIN_COST);
    ROS_INFO_STREAM("RES " << RES);
    ROS_INFO_STREAM("W " << W);
    ROS_INFO_STREAM("H " << H);
    ROS_INFO_STREAM("FIXED_FRAME " << FIXED_FRAME);
    ROS_INFO_STREAM("--------------------------");
}

/*
update map with occupied points from occupancy map

input:
    occPoints -> vector of vectors that contains occupied grid coordinates
*/
void Costmap::update(std::vector<std::vector<int>> occPoints)
{
    // we clear map before updating it
    clear();

    for(int i = 0, n = occPoints.size(); i< n; i++)
    {
        int x = occPoints[i][0];
        int y = occPoints[i][1];

        inflate(x, y);
    }
}

/*
inflate given grid coordinate

input:
    x, y -> coordinate of obstackle
*/
void Costmap::inflate(int x,int y)
{   
    int cost = MAX_COST;

    // we put variable cost each radius from total radius to center
    for(int r = 0; r <= totalCostGridR; r++)
    {
        if(r > lethalCostGridR)
        {
            cost = std::max(MIN_COST, cost-costDecay);
        }

        //fill_circle(x, y, r, cost); 
        fill_circle_midpoint(x, y, r, cost); // -> bu daha iyi
    }
}

/*
fill inflation circle using bresenham circle algorithm

inputs:
    x_, y_ -> center of circle 
    r    -> radius of circle
    cost -> inflation cost will be give to circle coordinates
*/
void Costmap::fill_circle(int x_, int y_, int r, int cost)
{
    int x = 0;
    int y = r;

    int d = 3 - 2*r;

    fill(x, y, x_, y_, cost);

    while(x <= y)
    {
        x++;

        if(d < 0)
        {
            d = d + 4*x + 6;
        }
        else
        {
            y--;
            d = d + 4*(x-y) + 10;
        }

        fill(x, y, x_, y_, cost);
    }
}

/*
fill inflation circle using midpoint circle algorithm
NOTE: good alternative to bresenham

inputs:
    x_, y_ -> center of circle 
    r    -> radius of circle
    cost -> inflation cost will be give to circle coordinates
*/
void Costmap::fill_circle_midpoint(int x_, int y_, int r, int cost)
{
    int x = r;
    int y = 0;

    int p = 1 - r;

    fill(x, y, x_, y_, cost);

    while(x > y)
    {
        y++;

        if(p <= 0)
        {
            p = p + 2*y + 1;
        }
        else
        {
            x--;
            p = p + 2*y + 1;
        }

        fill(x, y, x_, y_, cost);
    }
}

/*
This is helper function of fill_cirle. In a nutshell fill_circle
only calculates one octane of circle. so we need to put pixel to 8
symmetric octanes in order to contruct circle

input:
    x, y   -> calculated relative x y octane coordinate
    x_, y_ -> center of the radius
    cost   -> desired cost value
*/
void Costmap::fill(int x, int y, int x_, int y_, int cost)
{
    putCost(x+x_, y+y_, cost); //1
    putCost(y+x_, x+y_, cost); //2
    putCost(y+x_, y_-x, cost); //3
    putCost(x+x_, y_-y, cost); //4
    putCost(x_-x, y_-y, cost); //5
    putCost(x_-y, y_-x, cost); //6
    putCost(x_-y, y_+x, cost); //7
    putCost(x_-x, y_+y, cost); //8
}

/*
put cost value to given grid coordinate

input:
    x, y -> coordinate
    cost -> desired cost value
*/
void Costmap::putCost(int x, int y, int cost)
{
    if(inRange(x, y))
    {
        int idx = I(x, y);
        int cost_ = grid.data[idx];
        cost = std::max(cost_, cost);
        grid.data[idx] = cost;
    }
}
