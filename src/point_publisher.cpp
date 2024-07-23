#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class PointPublisher
{
public:
    PointPublisher()
    {
        point_pub = n.advertise<std_msgs::Float32MultiArray>("clicked_points", 10);
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        sub = n.subscribe("/clicked_point", 10, &PointPublisher::pointCallback, this);
        timer = n.createTimer(ros::Duration(0.1), &PointPublisher::timerCallback, this); // 타이머 콜백 설정
    }

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        points.push_back(msg->point);
        publishPoints();
        printPoints();  // 선택된 점들의 좌표값을 출력
    }

    void publishPoints()
    {
        std_msgs::Float32MultiArray points_msg;
        for (const auto& point : points)
        {
            points_msg.data.push_back(point.x);
            points_msg.data.push_back(point.y);
            points_msg.data.push_back(point.z);
        }
        point_pub.publish(points_msg);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        publishMarker();
    }

    void publishMarker()
    {
        if (!points.empty())
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";  // 수정: "map"으로 변경
            marker.header.stamp = ros::Time::now();
            marker.ns = "points";
            marker.id = 1;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.color.a = 0.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker.points = points;
            marker_pub.publish(marker);
        }
    }

private:
    ros::NodeHandle n;
    ros::Publisher point_pub;
    ros::Publisher marker_pub;
    ros::Subscriber sub;
    ros::Timer timer; // 타이머 선언
    std::vector<geometry_msgs::Point> points;

    void printPoints()  // 선택된 점들의 좌표값을 출력하는 함수
    {
        ROS_INFO("Selected Points:");
        for (size_t i = 0; i < points.size(); ++i)
        {
            ROS_INFO("Point %zu: [x: %f, y: %f, z: %f]", i, points[i].x, points[i].y, points[i].z);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_publisher");
    PointPublisher pp;
    ros::spin();
}
