#ifndef NRS_VISUALIZATION_H
#define NRS_VISUALIZATION_H

#include <ros/ros.h>
#include <nrs_path/Waypoints.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <array>
#include <string>

class nrs_visualization
{
public:
    // 기본 생성자: NodeHandle 없이 생성 (퍼블리셔들은 초기화되지 않음)
    nrs_visualization();
    ~nrs_visualization();

    // init() 함수를 호출해서 ros::NodeHandle를 전달하면 퍼블리셔들이 초기화됩니다.
    void init(ros::NodeHandle &nh);

    // Callback 함수들
    void waypointsCallback(const nrs_path::Waypoints::ConstPtr &msg);
    void visualizeClickedPoint(const geometry_msgs::PointStamped::ConstPtr &msg);
    void deleteMarkers();

    // 시각화 함수: 경로(Line Strip) 시각화
    void visualizePath(const std::vector<geometry_msgs::Point> &path, const std::string &ns, int id,
                       float r, float g, float b, float a);

private:
    ros::Publisher marker_pub1;
    ros::Publisher marker_pub2;
    ros::Publisher clicked_marker_pub;
    std::vector<geometry_msgs::Point> path;
    int clicked_marker_id;
};

#endif // NRS_VISUALIZATION_H
