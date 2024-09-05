#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nrs_path_planning/Waypoints.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <limits>

class visualization
{
public:
    // visualize ContolPoints
    void visualizeControlPoints(const Eigen::Vector3d &point, visualization_msgs::MarkerArray &marker_array, int &marker_id, const std::string &ns, const std::string &frame_id = "base_link");
    void visualizePath(const std::vector<geometry_msgs::Point> &path, const std::string &ns, int id, float r, float g, float b, float a);
    void visualizeWaypointsAxes(const std::vector<nrs_path_planning::Waypoint> &waypoints);
};