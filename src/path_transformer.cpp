#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <nrs_ver2/Waypoint.h>
#include <nrs_ver2/Waypoints.h>

using namespace std;
using namespace Eigen;

// RPY 값을 변환 행렬로 변환
Matrix4d RPYToMatrix(double roll, double pitch, double yaw)
{
    Matrix4d transform = Matrix4d::Identity();

    // Rotation matrices
    Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    Matrix3d rotation = Rz * Ry * Rx;

    transform.block<3, 3>(0, 0) = rotation;
    return transform;
}

// 변환 행렬로부터 RPY를 추출
Vector3d MatrixToRPY(const Matrix3d &rotation)
{
    double roll = atan2(rotation(2, 1), rotation(2, 2));
    double pitch = atan2(-rotation(2, 0), sqrt(pow(rotation(2, 1), 2) + pow(rotation(2, 2), 2)));
    double yaw = atan2(rotation(1, 0), rotation(0, 0));

    return Vector3d(roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    // ROS 초기화
    ros::init(argc, argv, "path_transformer_node");
    ros::NodeHandle nh;

    // ROS 파라미터 읽기
    string source_file, target_file;

    nh.getParam("path_transformer_node/source_file", source_file);
    nh.getParam("path_transformer_node/target_file", target_file);

    // `waypoints_interpolator_node/save_complete`가 true인지 확인
    bool save_complete = false;
    ros::Rate rate(1); // 1Hz로 체크
    while (ros::ok())
    {
        nh.getParam("waypoints_interpolator_node/save_complete", save_complete);
        if (save_complete)
        {
            ROS_INFO("save_complete is true. Starting transformation...");
            break;
        }
        else
        {
            // ROS_INFO("Waiting for save_complete to be true...");
            rate.sleep();
        }
    }

    // 변환 행렬 정의 (사용자 지정)
    Matrix4d transformation = Matrix4d::Identity();
    transformation << 0.83487365, 0.55008218, 0.01988921, 0.5442881,
        -0.550408, 0.83467687, 0.01911928, 0.03528842,
        -0.00608389, -0.02690936, 0.99961936, 0.22049255,
        0.0, 0.0, 0.0, 1.0;

    // ROS Publisher 생성
    ros::Publisher waypoint_pub = nh.advertise<nrs_ver2::Waypoints>("transformed_waypoints", 10);
    nrs_ver2::Waypoints waypoints_msg, send_waypoints_msg;
    // 파일 열기
    ifstream inputFile(source_file);
    ofstream outputFile(target_file);

    string line;
    while (getline(inputFile, line))
    {
        istringstream iss(line);
        double x, y, z, roll, pitch, yaw, forceX, forceY, forceZ;

        // 데이터를 파싱
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw >> forceX >> forceY >> forceZ))
        {
            ROS_WARN("Failed to parse a line in the source file. Skipping...");
            continue;
        }

        // Position 변환
        Vector4d position(x, y, z, 1.0);
        position = transformation * position;

        // Orientation 변환
        Matrix4d rotationMatrix = RPYToMatrix(roll, pitch, yaw);
        Matrix4d transformedRotation = transformation * rotationMatrix;
        Vector3d transformedRPY = MatrixToRPY(transformedRotation.block<3, 3>(0, 0));

        // Force 변환
        Vector3d force(forceX, forceY, forceZ);
        force = transformation.block<3, 3>(0, 0) * force;

        // 결과를 출력 파일에 저장
        outputFile << position(0) << " " << position(1) << " " << position(2) << " "
                   << transformedRPY(0) << " " << transformedRPY(1) << " " << transformedRPY(2) << " "
                   << force(0) << " " << force(1) << " " << force(2) << endl;
        // Waypoint 메시지 생성

        nrs_ver2::Waypoint waypoint_msg;

        waypoint_msg.pose.position.x = position(0);
        waypoint_msg.pose.position.y = position(1);
        waypoint_msg.pose.position.z = position(2);

        waypoint_msg.pose.orientation.x = sin(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2) -
                                          cos(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2);
        waypoint_msg.pose.orientation.y = cos(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2) +
                                          sin(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2);
        waypoint_msg.pose.orientation.z = cos(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2) -
                                          sin(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2);
        waypoint_msg.pose.orientation.w = cos(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2) +
                                          sin(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2);

        // 메시지를 Waypoints 배열에 추가
        waypoints_msg.waypoints.push_back(waypoint_msg);

        send_waypoints_msg = waypoints_msg;
    }

    inputFile.close();
    outputFile.close();
    std::cout << "waypoints_msg size: " << waypoints_msg.waypoints.size() << std::endl;
    // Waypoints 메시지 퍼블리시
    waypoint_pub.publish(send_waypoints_msg);

    ROS_INFO("Transformation completed and published! Result saved in %s", target_file.c_str());
    ros::spin();
    return 0;
}
