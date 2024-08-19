// path_visualizer.cpp

#include <ros/ros.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>

// 전역 변수 선언
ros::Publisher marker_pub;

void visualizePath(const std::vector<geometry_msgs::Point> &path, const std::string &ns, int id, float r, float g, float b, float a)
{
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "base_link";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = ns;
    path_marker.id = id;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.005;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;
    path_marker.color.a = a;

    for (const auto &point : path)
    {
        path_marker.points.push_back(point);
    }

    marker_pub.publish(path_marker);
}

void visualizeWaypointsAxes(const std::vector<nrs_vision_rviz::Waypoint> &waypoints)
{
    double axis_length = 0.01; // 축의 고정 길이

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        const auto &waypoint = waypoints[i];
        const auto &origin = waypoint.point;
        const auto &z_axis = waypoint.normal;

        // 다음 점을 기준으로 x축 방향을 설정 (path 진행 방향)
        geometry_msgs::Point x_axis_end;
        if (i < waypoints.size() - 1)
        {
            const auto &next_point = waypoints[i + 1].point;

            // x축 벡터 계산
            geometry_msgs::Point x_axis;
            x_axis.x = next_point.x - origin.x;
            x_axis.y = next_point.y - origin.y;
            x_axis.z = next_point.z - origin.z;

            // 벡터를 정규화
            double length = std::sqrt(x_axis.x * x_axis.x + x_axis.y * x_axis.y + x_axis.z * x_axis.z);
            x_axis.x = (x_axis.x / length) * axis_length;
            x_axis.y = (x_axis.y / length) * axis_length;
            x_axis.z = (x_axis.z / length) * axis_length;

            // x_axis_end 포인트 계산
            x_axis_end.x = origin.x + x_axis.x;
            x_axis_end.y = origin.y + x_axis.y;
            x_axis_end.z = origin.z + x_axis.z;
        }
        else
        {
            // 마지막 점인 경우, x축 방향을 0으로 설정
            x_axis_end = origin;
        }

        // y축을 z축과 x축의 외적(cross product)으로 계산
        geometry_msgs::Point y_axis_end;
        y_axis_end.x = origin.x + (-z_axis.y * (x_axis_end.z - origin.z) - -z_axis.z * (x_axis_end.y - origin.y));
        y_axis_end.y = origin.y + (-z_axis.z * (x_axis_end.x - origin.x) - -z_axis.x * (x_axis_end.z - origin.z));
        y_axis_end.z = origin.z + (-z_axis.x * (x_axis_end.y - origin.y) - -z_axis.y * (x_axis_end.x - origin.x));

        // z축 끝 포인트 계산 (정규화)
        geometry_msgs::Point z_axis_end;
        double z_length = std::sqrt(-z_axis.x * -z_axis.x + -z_axis.y * -z_axis.y + -z_axis.z * -z_axis.z);
        z_axis_end.x = origin.x + (-z_axis.x / z_length) * axis_length;
        z_axis_end.y = origin.y + (-z_axis.y / z_length) * axis_length;
        z_axis_end.z = origin.z + (-z_axis.z / z_length) * axis_length;

        // 마커 생성
        visualization_msgs::Marker x_axis_marker, y_axis_marker, z_axis_marker;

        x_axis_marker.header.frame_id = y_axis_marker.header.frame_id = z_axis_marker.header.frame_id = "base_link";
        x_axis_marker.header.stamp = y_axis_marker.header.stamp = z_axis_marker.header.stamp = ros::Time::now();
        x_axis_marker.ns = y_axis_marker.ns = z_axis_marker.ns = "waypoint_axes";
        x_axis_marker.action = y_axis_marker.action = z_axis_marker.action = visualization_msgs::Marker::ADD;
        x_axis_marker.type = y_axis_marker.type = z_axis_marker.type = visualization_msgs::Marker::ARROW;
        x_axis_marker.scale.x = y_axis_marker.scale.x = z_axis_marker.scale.x = 0.0005; // 화살표 몸통의 두께
        x_axis_marker.scale.y = y_axis_marker.scale.y = z_axis_marker.scale.y = 0.001;  // 화살표 머리의 두께
        x_axis_marker.scale.z = y_axis_marker.scale.z = z_axis_marker.scale.z = 0.001;  // 화살표 머리의 길이
        x_axis_marker.color.a = y_axis_marker.color.a = z_axis_marker.color.a = 1.0;

        // x축 색상 (빨간색)
        x_axis_marker.color.r = 1.0;
        x_axis_marker.color.g = 0.0;
        x_axis_marker.color.b = 0.0;

        // y축 색상 (녹색)
        y_axis_marker.color.r = 0.0;
        y_axis_marker.color.g = 1.0;
        y_axis_marker.color.b = 0.0;

        // z축 색상 (파란색)
        z_axis_marker.color.r = 0.0;
        z_axis_marker.color.g = 0.0;
        z_axis_marker.color.b = 1.0;

        // 각 마커의 고유 ID 설정
        x_axis_marker.id = i * 3;
        y_axis_marker.id = i * 3 + 1;
        z_axis_marker.id = i * 3 + 2;

        // 마커 시작점과 끝점 설정
        x_axis_marker.points.push_back(origin);
        x_axis_marker.points.push_back(x_axis_end);

        y_axis_marker.points.push_back(origin);
        y_axis_marker.points.push_back(y_axis_end);

        z_axis_marker.points.push_back(origin);
        z_axis_marker.points.push_back(z_axis_end);

        // 마커 퍼블리시
        marker_pub.publish(x_axis_marker);
        marker_pub.publish(y_axis_marker);
        marker_pub.publish(z_axis_marker);
    }
}

void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{
    std::vector<geometry_msgs::Point> path;
    for (const auto &waypoint : msg->waypoints)
    {
        path.push_back(waypoint.point);
    }

    visualizePath(path, "geodesic_path", 0, 0.0, 1.0, 0.0, 1.0);
    //visualizeWaypointsAxes(msg->waypoints);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_visualizer");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber waypoints_sub = nh.subscribe("waypoints_with_normals", 10, waypointsCallback);
    
    ros::spin();
    return 0;
}
