#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <std_msgs/String.h>

// 전역 변수
ros::Publisher marker_pub;
int marker_id = 0;

// 클릭된 점을 처리하는 콜백 함수
void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

// 마커 삭제 함수
void deleteMarkers();

// 키보드 입력 처리 콜백 함수
void keyboardCallback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clicked_point_visualizer");
    ros::NodeHandle nh;

    // Publisher 설정 (RViz에서 시각화를 위한 Marker 토픽)
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // /clicked_point 토픽을 구독하고, 클릭된 점을 처리하는 콜백 함수 등록
    ros::Subscriber sub = nh.subscribe("/clicked_point", 10, pointCallback);

    // 키보드 명령어를 구독하는 Subscriber
    ros::Subscriber keyboard_sub = nh.subscribe("moveit_command", 10, keyboardCallback);

    ros::spin(); // 콜백 함수 대기

    return 0;
}

// 클릭된 점을 처리하는 콜백 함수
void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    // 클릭된 점 정보를 로그로 출력
    ROS_INFO("Received point: [%.6f, %.6f, %.6f]", msg->point.x, msg->point.y, msg->point.z);

    // Marker 메시지 생성
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link"; // 필요에 맞게 frame_id를 설정
    marker.header.stamp = ros::Time::now();
    marker.ns = "clicked_points";
    marker.id = marker_id++;                          // 고유 ID
    marker.type = visualization_msgs::Marker::SPHERE; // 구 형태로 점을 시각화
    marker.action = visualization_msgs::Marker::ADD;

    // 클릭된 점의 좌표 설정
    marker.pose.position.x = msg->point.x;
    marker.pose.position.y = msg->point.y;
    marker.pose.position.z = msg->point.z;

    // 크기 설정 (지름)
    marker.scale.x = 0.007;
    marker.scale.y = 0.007;
    marker.scale.z = 0.007;

    // 색상 설정 (빨간색, 불투명)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Marker 퍼블리시
    marker_pub.publish(marker);
}

// 마커 삭제 함수
void deleteMarkers()
{
    // 마커 삭제를 위한 메시지 생성
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "base_link";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "clicked_points";                          // 삭제하려는 마커와 동일한 네임스페이스 사용
    delete_marker.id = 0;                                         // 삭제하려는 마커 ID (필요시 여러 마커 삭제)
    delete_marker.action = visualization_msgs::Marker::DELETEALL; // 모든 마커 삭제

    // 삭제 명령을 퍼블리시
    marker_pub.publish(delete_marker);

    ROS_INFO("All markers deleted");
}

// 키보드 입력 처리 콜백 함수
void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "reset")
    {
        // 마커 삭제 호출
        deleteMarkers();

        ROS_INFO("Markers cleared on 'reset' command");
    }
}