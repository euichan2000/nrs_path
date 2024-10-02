// path_visualizer.cpp

#include <ros/ros.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 전역 변수 선언
ros::Publisher marker_pub1;
ros::Publisher marker_pub2;

std::vector<geometry_msgs::Point> path;

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
    path_marker.scale.x = 0.003;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;
    path_marker.color.a = a;

    for (const auto &point : path)
    {
        path_marker.points.push_back(point);
    }

    marker_pub1.publish(path_marker);
}

void visualizePoints(const std::vector<geometry_msgs::Point> &points, const std::string &ns)
{
    int id = 0; // 각 마커의 고유 ID

    // 색상 설정: b0 = 노랑, b1 = 초록, b2 = 파랑, b3 = 검정
    std::vector<std::array<float, 4>> colors = {
        {1.0, 1.0, 0.0, 1.0}, // b0: 노랑 (r, g, b, a)
        {0.0, 1.0, 0.5, 1.0}, // b1: 초록
        {0.0, 0.0, 1.0, 1.0}, // b2: 파랑
        {0.0, 0.0, 0.0, 1.0}  // b3: 검정
    };

    // 각 포인트에 대해 마커 생성 및 퍼블리시
    for (size_t i = 0; i < points.size(); ++i)
    {
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = "base_link";             // 프레임 ID 설정
        sphere_marker.header.stamp = ros::Time::now();           // 타임스탬프 설정
        sphere_marker.ns = ns;                                   // 네임스페이스 설정
        sphere_marker.id = id++;                                 // 고유한 ID 설정
        sphere_marker.type = visualization_msgs::Marker::SPHERE; // 마커 타입을 구로 설정
        sphere_marker.action = visualization_msgs::Marker::ADD;  // 마커 추가

        // 마커의 위치 설정
        sphere_marker.pose.position = points[i]; // points[i]의 위치를 마커 위치로 설정
        sphere_marker.pose.orientation.w = 1.0;  // 기본값으로 설정

        // Sphere 크기 설정
        sphere_marker.scale.x = 0.01; // X축 크기
        sphere_marker.scale.y = 0.01; // Y축 크기
        sphere_marker.scale.z = 0.01; // Z축 크기

        // 색상 설정: 4개마다 반복 (b0, b1, b2, b3)
        int color_index = i % 4; // 0, 1, 2, 3 반복
        sphere_marker.color.r = colors[color_index][0];
        sphere_marker.color.g = colors[color_index][1];
        sphere_marker.color.b = colors[color_index][2];
        sphere_marker.color.a = colors[color_index][3]; // 투명도는 항상 1.0

        // 마커 퍼블리시
        marker_pub2.publish(sphere_marker);
    }
}

// Waypoints 시각화 함수
void visualizeWaypointsAxes(const std::vector<nrs_vision_rviz::Waypoint> &waypoints)
{
    double axis_length = 0.01; // 축의 길이 설정

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        const auto &waypoint = waypoints[i];
        const auto &origin = waypoint.pose.position; // 원점

        // Quaternion을 사용하여 Rotation Matrix를 생성
        tf2::Quaternion q(
            waypoint.pose.orientation.x,
            waypoint.pose.orientation.y,
            waypoint.pose.orientation.z,
            waypoint.pose.orientation.w);

        // 쿼터니언을 Rotation Matrix로 변환
        tf2::Matrix3x3 rotation_matrix(q);

        // X, Y, Z 축 방향 설정
        tf2::Vector3 x_axis = rotation_matrix * tf2::Vector3(axis_length, 0, 0);
        tf2::Vector3 y_axis = rotation_matrix * tf2::Vector3(0, axis_length, 0);
        tf2::Vector3 z_axis = rotation_matrix * tf2::Vector3(0, 0, axis_length);

        // X축, Y축, Z축 끝 좌표 계산
        geometry_msgs::Point x_axis_end, y_axis_end, z_axis_end;

        x_axis_end.x = origin.x + x_axis.getX();
        x_axis_end.y = origin.y + x_axis.getY();
        x_axis_end.z = origin.z + x_axis.getZ();

        y_axis_end.x = origin.x + y_axis.getX();
        y_axis_end.y = origin.y + y_axis.getY();
        y_axis_end.z = origin.z + y_axis.getZ();

        z_axis_end.x = origin.x + z_axis.getX();
        z_axis_end.y = origin.y + z_axis.getY();
        z_axis_end.z = origin.z + z_axis.getZ();

        // 마커 생성 (X, Y, Z 축 각각)
        visualization_msgs::Marker x_axis_marker, y_axis_marker, z_axis_marker;

        x_axis_marker.header.frame_id = y_axis_marker.header.frame_id = z_axis_marker.header.frame_id = "base_link";
        x_axis_marker.header.stamp = y_axis_marker.header.stamp = z_axis_marker.header.stamp = ros::Time::now();
        x_axis_marker.ns = y_axis_marker.ns = z_axis_marker.ns = "waypoint_axes";
        x_axis_marker.action = y_axis_marker.action = z_axis_marker.action = visualization_msgs::Marker::ADD;
        x_axis_marker.type = y_axis_marker.type = z_axis_marker.type = visualization_msgs::Marker::ARROW;

        // 화살표 크기 설정
        x_axis_marker.scale.x = y_axis_marker.scale.x = z_axis_marker.scale.x = 0.0005; // 화살표 몸통 두께
        x_axis_marker.scale.y = y_axis_marker.scale.y = z_axis_marker.scale.y = 0.001;  // 화살표 머리 두께
        x_axis_marker.scale.z = y_axis_marker.scale.z = z_axis_marker.scale.z = 0.001;  // 화살표 머리 길이
        x_axis_marker.color.a = y_axis_marker.color.a = z_axis_marker.color.a = 1.0;

        // X축 색상 (빨간색)
        x_axis_marker.color.r = 1.0;
        x_axis_marker.color.g = 0.0;
        x_axis_marker.color.b = 0.0;

        // Y축 색상 (녹색)
        y_axis_marker.color.r = 0.0;
        y_axis_marker.color.g = 1.0;
        y_axis_marker.color.b = 0.0;

        // Z축 색상 (파란색)
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
        marker_pub1.publish(x_axis_marker);
        marker_pub1.publish(y_axis_marker);
        marker_pub1.publish(z_axis_marker);
    }
}

void deleteMarkers()
{
    // 마커 삭제를 위한 메시지 생성
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "base_link";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "geodesic_path"; // 삭제하려는 마커와 동일한 네임스페이스 사용
    delete_marker.id = 0;               // 삭제하려는 마커 ID (모든 마커를 삭제하려면 범위를 설정할 수 있습니다)
    delete_marker.action = visualization_msgs::Marker::DELETE;

    // 삭제 명령을 퍼블리시
    marker_pub1.publish(delete_marker);

    ROS_INFO("Markers deleted");
}

void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{

    for (const auto &waypoint : msg->waypoints)
    {
        path.push_back(waypoint.pose.position);
        //std::cout << "waypoint: " << waypoint.pose.position.x << " " << waypoint.pose.position.y << " " << waypoint.pose.position.z << std::endl;
    }

    visualizePath(path, "geodesic_path", 0, 0.0, 1.0, 0.0, 1.0);
    // visualizeWaypointsAxes(msg->waypoints);
}

void controlpointsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    std::vector<geometry_msgs::Point> control_points;

    // msg->data는 1차원 배열로 [x1, y1, z1, x2, y2, z2, ...] 형태로 구성되어 있음
    if (msg->data.size() % 3 != 0)
    {
        ROS_ERROR("Received control points data size is not divisible by 3. It must contain x, y, z coordinates.");
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i += 3)
    {
        geometry_msgs::Point point;
        point.x = msg->data[i];
        point.y = msg->data[i + 1];
        point.z = msg->data[i + 2];
        //std::cout << "controlpoint:  " << point.x << " " << point.y << " " << point.z << std::endl;
        control_points.push_back(point);
    }

    // visualizePoints(control_points, "control_points");
}

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{

    if (msg->data == "reset")
    {
        path.clear();
        deleteMarkers(); // 마커 삭제 호출

        ROS_INFO("waypoints cleared");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_visualizer");
    ros::NodeHandle nh;

    marker_pub1 = nh.advertise<visualization_msgs::Marker>("visualization_marker_path", 10);
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("visualization_marker_point", 10);
    ros::Subscriber waypoints_sub = nh.subscribe("interpolated_waypoints_with_normals", 10, waypointsCallback);
    ros::Subscriber controlpoints_sub = nh.subscribe("control_points", 10, controlpointsCallback);
    ros::Subscriber keyboard_sub = nh.subscribe("nrs_command", 10, keyboardCallback);
    ros::spin();
    return 0;
}
