#include "nrs_visualization.h"

// 기본 생성자: 초기화 작업은 init()에서 수행
nrs_visualization::nrs_visualization() : clicked_marker_id(0)
{
    // 퍼블리셔들은 아직 초기화하지 않음.
}

nrs_visualization::~nrs_visualization()
{
    // 필요 시 자원 해제 (현재 특별한 작업 없음)
}

// init() 함수: NodeHandle를 받아 퍼블리셔들을 초기화합니다.
void nrs_visualization::init(ros::NodeHandle &nh)
{
    marker_pub1 = nh.advertise<visualization_msgs::Marker>("visualization_marker_path", 10);
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("visualization_marker_point", 10);
}

// 지정된 path를 LINE_STRIP 타입 마커로 시각화
void nrs_visualization::visualizePath(const std::vector<geometry_msgs::Point> &path,
                                      const std::string &ns, int id,
                                      float r, float g, float b, float a)
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

    for (const auto &pt : path)
    {
        path_marker.points.push_back(pt);
    }
    marker_pub1.publish(path_marker);
}

// 마커 삭제 함수: 두 퍼블리셔에 DELETEALL 메시지를 보내고, 내부의 path도 clear합니다.
void nrs_visualization::deleteMarkers()
{
    path.clear();
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "base_link";
    delete_marker.header.stamp = ros::Time::now();
    // 각 네임스페이스에 대해 삭제 메시지 발행
    delete_marker.ns = "geodesic_path";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub1.publish(delete_marker);

    delete_marker.ns = "clicked_points";
    marker_pub2.publish(delete_marker);

    ROS_INFO("All markers and internal path cleared");
}

// Waypoints 메시지 콜백: 수신한 웨이포인트에서 위치만 추출해 path 멤버에 저장하고 시각화 호출
void nrs_visualization::waypointsCallback(const nrs_path::Waypoints::ConstPtr &msg)
{
    path.clear();
    for (const auto &wp : msg->waypoints)
    {
        geometry_msgs::Point pt;
        pt.x = wp.x;
        pt.y = wp.y;
        pt.z = wp.z;
        path.push_back(pt);
    }
    visualizePath(path, "geodesic_path", 0, 0.0f, 1.0f, 0.0f, 1.0f);
}

// 클릭된 점을 시각화하는 콜백 함수: /clicked_point 토픽 메시지 처리
void nrs_visualization::visualizeClickedPoint(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clicked_points";
    marker.id = clicked_marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = msg->point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub2.publish(marker);
}
