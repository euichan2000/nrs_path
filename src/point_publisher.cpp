#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

ros::NodeHandle n;
ros::Publisher point_pub;
ros::Publisher marker_pub;
ros::Subscriber sub;
ros::Timer timer; // 타이머 선언
std::vector<geometry_msgs::Point> points;

void publishPoints()
{
    std_msgs::Float32MultiArray points_msg;
    for (const auto &point : points)
    {
        points_msg.data.push_back(point.x);
        points_msg.data.push_back(point.y);
        points_msg.data.push_back(point.z);
    }
    point_pub.publish(points_msg);
}


void publishMarker()
{
    if (!points.empty())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "points";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.007;
        marker.scale.y = 0.007;
        marker.color.a = 0.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.points = points;
        marker_pub.publish(marker);
    }
}
void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    points.push_back(msg->point);
    publishPoints();
    // printPoints();  // 선택된 점들의 좌표값을 출력
}
void timerCallback(const ros::TimerEvent &)
{
    publishMarker();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "point_publisher");

    point_pub = n.advertise<std_msgs::Float32MultiArray>("clicked_points", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    sub = n.subscribe("/clicked_point", 10, pointCallback);
    timer = n.createTimer(ros::Duration(0.1), timerCallback); // 타이머 콜백 설정

    ros::spin();
}
