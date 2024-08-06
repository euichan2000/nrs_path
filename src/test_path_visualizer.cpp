#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>

// Global vector to store clicked points
std::vector<geometry_msgs::Point> clicked_points;

// Function to generate a smooth path using Catmull-Rom splines
std::vector<geometry_msgs::Point> generateSmoothPath(const std::vector<geometry_msgs::Point>& points)
{
    std::vector<geometry_msgs::Point> smooth_path;
    if (points.size() < 4) return smooth_path; // Need at least 4 points for Catmull-Rom spline

    for (size_t i = 1; i < points.size() - 2; ++i)
    {
        geometry_msgs::Point p0 = points[i - 1];
        geometry_msgs::Point p1 = points[i];
        geometry_msgs::Point p2 = points[i + 1];
        geometry_msgs::Point p3 = points[i + 2];

        for (double t = 0; t <= 1; t += 0.05)
        {
            double t2 = t * t;
            double t3 = t2 * t;

            geometry_msgs::Point p;
            p.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
            p.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
            p.z = 0.5 * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 + (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3);

            smooth_path.push_back(p);
        }
    }

    return smooth_path;
}

// Callback function for clicked points
void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    clicked_points.push_back(msg->point);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_blending_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Rate r(30);

    while (ros::ok())
    {
        ros::spinOnce();

        // Generate smooth path
        std::vector<geometry_msgs::Point> smooth_path = generateSmoothPath(clicked_points);

        // Publish markers for smooth path
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "base_link";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path";
        path_marker.id = 1;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.005;
        path_marker.color.b = 1.0;
        path_marker.color.a = 1.0;

        for (const auto& point : smooth_path)
        {
            path_marker.points.push_back(point);
        }

        marker_pub.publish(path_marker);

        r.sleep();
    }

    return 0;
}
