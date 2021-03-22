/*
    Date 24.01.2021
    Author: Berke Algül
*/

#include <chrono>
#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "map/occmap.h"
#include "map/costmap.h"

int COST_UPDATE_RATE;
bool scan_subbed, odom_subbed;
std::string occ_topic, cost_topic, scan_topic, odom_topic;

Costmap *costmap;
OccupancyMap *occmap;

void odomCallback(const nav_msgs::Odometry::ConstPtr&);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");

    ros::NodeHandle nh("~");

    ros::Subscriber sub_odom;
    ros::Subscriber sub_scan;
    ros::Publisher occ_pub;
    ros::Publisher cost_pub;

    nh.getParam("/RoverMapping/Costmap/publish_topic", cost_topic);
    nh.getParam("/RoverMapping/OccupancyMap/publish_topic", occ_topic);
    nh.getParam("/RoverMapping/costmap_update_rate", COST_UPDATE_RATE);
    nh.getParam("/RoverMapping/scan_topic", scan_topic);
    nh.getParam("/RoverMapping/odom_topic", odom_topic);

    sub_odom = nh.subscribe(odom_topic, 10, odomCallback);
    sub_scan = nh.subscribe(scan_topic, 10, scanCallback);
    occ_pub = nh.advertise<nav_msgs::OccupancyGrid>(occ_topic, 10);
    cost_pub = nh.advertise<nav_msgs::OccupancyGrid>(cost_topic, 10);

    occmap = new OccupancyMap(nh);
    costmap = new Costmap(nh);

    ROS_INFO("Mapping Initialized");

    // countdown to costmap update
    int cost_cd = 0;

    while(ros::ok())
    {
        ros::spinOnce();

        //make sure we subscribed both scan and odom
        if(!(odom_subbed && scan_subbed))
        {
            continue;
        }

        odom_subbed = false;
        scan_subbed = false;

        auto time_b = ros::Time::now();

        // Map update area
        occmap->update();

        // update costmap
        if(cost_cd == 0)
        {
            cost_cd = COST_UPDATE_RATE;
            costmap->update(occmap->occupiedPoints);
            ROS_INFO("Costmap updated");
        }

        cost_cd--;

        double dt = (ros::Time::now() - time_b).toSec();

        // publish area
        occ_pub.publish(occmap->msg());
        cost_pub.publish(costmap->msg());

        ROS_INFO_STREAM("Updated in (sec): " << dt);
    }

    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    //ROS_INFO("ODOM SUBBED");

    occmap->x = odom->pose.pose.position.x;
    occmap->y = odom->pose.pose.position.y;

    // covert quaternion to euler and extract yaw
    tf::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(q); // q -> matrix
    double r, p, y;
    m.getRPY(r, p, y); // matrx -> euler
    
    occmap->yaw = (float)y;

    odom_subbed = true;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //ROS_INFO("SCAN SUBBED");

    occmap->min_a  = scan->angle_min;      
    occmap->max_a  = scan->angle_max;    
    occmap->da     = scan->angle_increment; 
    occmap->ranges = scan->ranges;
    occmap->range_max = scan->range_max;

    scan_subbed = true;
}
