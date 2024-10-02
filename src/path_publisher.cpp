#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <sstream>

void sendFile(const std::string &file_path)
{
    // ROS 노드 핸들 생성
    ros::NodeHandle nh;

    // 토픽 퍼블리셔 생성
    ros::Publisher file_pub = nh.advertise<std_msgs::String>("path_publisher", 10);

    // 파일 열기
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    // 파일 데이터 읽기
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string file_data = buffer.str();

    // 파일을 닫기
    file.close();

    // 파일 데이터를 퍼블리시
    std_msgs::String msg;
    msg.data = file_data;
    ROS_INFO("Sending file data...");
    file_pub.publish(msg);
    ROS_INFO("File data sent.");

    // 파일 데이터 전송 시간을 주기 위해 ROS 루프 실행
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "file_transfer") // "file_transfer" 명령어를 받으면 파일을 전송
    {
        ROS_INFO("Received file transfer command. Sending file...");
        sendFile("/home/nrs/catkin_ws/src/nrs_vision_rviz/data/final_waypoints.txt"); // 전송할 파일 경로 지정
    }
    else
    {
        ROS_INFO("Received command: %s", msg->data.c_str());
    }
}

int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "path_publisher");
    
    ros::NodeHandle nh;
    // "nrs_command" 토픽 구독
    ros::Subscriber sub = nh.subscribe("nrs_command", 10, keyboardCallback);

    ROS_INFO("Waiting for commands...");

    ros::spin(); // 명령 대기 및 파일 전송
    return 0;
}
