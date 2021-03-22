#include <Eigen/Dense>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

#include "map/occmap.h"
#include "map/costmap.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "costmap");

    ros::NodeHandle nh;

    OccupancyMap map(nh);
    Costmap costmap(nh);
    

    std::vector<float> ranges;
    float r = 3;
    for(int i = 0; i < 360; i++)
    {
        r += 0.05;
        ranges.push_back(r);
    }

    float x = 0;
    float y = 0;
    float yaw = 0;

    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/rover_map", 10);
    ros::Publisher cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("/rover_cost_map", 10);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);

    while(ros::ok())
    {
        auto time_b = ros::Time::now();

        map.ranges = ranges;
        map.min_a = 0;
        map.da = M_PI/180;
        map.x = x;
        map.y = y;
        map.yaw = yaw;
        map.range_max = 50;

        map.update();
        costmap.update(map.occupiedPoints);

        pub.publish(map.msg());
        cost_pub.publish(costmap.msg());

        sensor_msgs::LaserScan scan;
        scan.header.frame_id = map.FIXED_FRAME;
        scan.range_max = 100;
        scan.range_min = 0.1;
        scan.angle_min = 0;      
        scan.angle_max = 2*M_PI;      
        scan.angle_increment = M_PI/180;

        for(int i = 0; i < 360; i++)
        {
            scan.ranges.push_back(ranges[i]);
        }
        pub2.publish(scan);

        //map.yaw = yaw;
        //yaw = yaw + .0005;

        /*
        map.x = x;
        x = x - 0.005;

        map.y = y;
        y = y - 0.005;*/

        double dt = (ros::Time::now() - time_b).toSec();
        ROS_INFO_STREAM("Updated in (sec): " << dt);
    }   

    /*
    while(ros::ok())
    {
        ros::spinOnce();
        
    }*/

    
    return 0;
}