#include "nrs_visualization.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <iostream>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_node_visualization");
    ros::NodeHandle nh;
    nrs_visualization visualizer;
    visualizer.init(nh);

    ros::Subscriber vis_waypoints_sub = nh.subscribe("interpolated_waypoints", 10, &nrs_visualization::waypointsCallback, &visualizer);

    ROS_INFO("nrs_node_visualiztion started.visualization functionalities are active.");

    ros::spin();
    return 0;
}
