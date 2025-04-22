#include "nrs_io.h"


// 파일 내용 초기화 함수 구현
void nrs_io::clearFile(const std::string &file_path)
{
    std::ofstream file(file_path, std::ofstream::trunc);
    if (!file.is_open())
        std::cerr << "Failed to open file: " << file_path << std::endl;
    else
        file.close();
}

// Waypoints 파일 저장 함수 구현
void nrs_io::saveWaypointsToFile(const nrs_path::Waypoints &final_waypoints,
                                 const std::string &file_path)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << file_path << std::endl;
        return;
    }
    for (size_t i = 0; i < final_waypoints.waypoints.size(); ++i)
    {
        const auto &wp = final_waypoints.waypoints[i];
        tf2::Quaternion q(wp.qx, wp.qy, wp.qz, wp.qw);
        double roll, pitch, yaw;
        roll = pitch = yaw = 0; // 임시 값
        n_math.quaternionToRPY(wp.qx, wp.qy, wp.qz, wp.qw, roll, pitch, yaw);

        file << wp.x << " " << wp.y << " " << wp.z << " "
             << roll << " " << pitch << " " << yaw << " "
             << wp.Fx << " " << wp.Fy << " " << wp.Fz << "\n";
    }
    file.close();
    std::cout << "Waypoints saved to " << file_path << std::endl;
}

// 파일 전송 함수 구현
void nrs_io::sendFile(const std::string &file_path, ros::Publisher &file_pub)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string file_data = buffer.str();
    file.close();
    std_msgs::String msg;
    msg.data = file_data;
    ROS_INFO("Sending file data...");
    file_pub.publish(msg);
    ROS_INFO("File data sent.");
}