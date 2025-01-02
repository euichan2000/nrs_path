#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <nrs_ver2/Waypoint.h>
#include <nrs_ver2/Waypoints.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

Matrix4d LoadTransformationMatrix(const std::string &filename)
{
    Matrix4d transformation = Matrix4d::Identity();

    // YAML 파일 읽기
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    // YAML 데이터 파싱
    YAML::Node doc = YAML::Load(fin);
    if (!doc["tf_matrix"])
    {
        std::cerr << "tf_matrix not found in YAML file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    // tf_matrix를 1D 배열로 읽기
    const auto &matrix_data = doc["tf_matrix"];
    if (matrix_data.size() != 16)
    {
        std::cerr << "tf_matrix must contain 16 elements, but found " << matrix_data.size() << std::endl;
        exit(EXIT_FAILURE);
    }

    // 1D 배열 데이터를 4x4 행렬로 변환
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            transformation(i, j) = matrix_data[i * 4 + j].as<double>();
        }
    }

    return transformation;
}

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
    string visual_source_file, visual_target_file, control_source_file, control_target_file;

    nh.getParam("path_transformer_node/visual_source_file", visual_source_file);
    nh.getParam("path_transformer_node/visual_target_file", visual_target_file);
    nh.getParam("path_transformer_node/control_source_file", control_source_file);
    nh.getParam("path_transformer_node/control_target_file", control_target_file);

    // tf_matrix를 LoadTransformationMatrix 함수로 로드
    Matrix4d transformation = LoadTransformationMatrix("/home/nrs_vision/catkin_ws/src/nrs_ver2/config/params.yaml");

    // 로드된 변환 행렬 출력
    cout << "Loaded Transformation Matrix:" << endl;
    cout << transformation << endl;

    // ROS Publisher 생성
    ros::Publisher waypoint_pub = nh.advertise<nrs_ver2::Waypoints>("final_waypoints", 10);
    nrs_ver2::Waypoints visual_waypoints_msg;

    // 파일 열기
    ifstream visual_inputFile(visual_source_file);
    ofstream visual_outputFile(visual_target_file);
    ifstream control_inputFile(control_source_file);
    ofstream control_outputFile(control_target_file);

    string line;
    //visual Waypoints 처리
    while (getline(visual_inputFile, line))
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

        // 결과를 출력 파일에 저장
        visual_outputFile << position(0) << " " << position(1) << " " << position(2) << " "
                          << transformedRPY(0) << " " << transformedRPY(1) << " " << transformedRPY(2) << " "
                          << forceX << " " << forceY << " " << forceZ << endl;

        // Waypoint 메시지 생성
        nrs_ver2::Waypoint visual_waypoint_msg;

        visual_waypoint_msg.pose.position.x = position(0);
        visual_waypoint_msg.pose.position.y = position(1);
        visual_waypoint_msg.pose.position.z = position(2);

        visual_waypoint_msg.pose.orientation.x = sin(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2) -
                                                 cos(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2);
        visual_waypoint_msg.pose.orientation.y = cos(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2) +
                                                 sin(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2);
        visual_waypoint_msg.pose.orientation.z = cos(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2) -
                                                 sin(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2);
        visual_waypoint_msg.pose.orientation.w = cos(transformedRPY(0) / 2) * cos(transformedRPY(1) / 2) * cos(transformedRPY(2) / 2) +
                                                 sin(transformedRPY(0) / 2) * sin(transformedRPY(1) / 2) * sin(transformedRPY(2) / 2);

        // 메시지를 Waypoints 배열에 추가
        visual_waypoints_msg.waypoints.push_back(visual_waypoint_msg);
    }

    visual_inputFile.close();
    visual_outputFile.close();
    // Control Waypoints 처리
    while (getline(control_inputFile, line))
    {
        istringstream iss(line);
        double x, y, z, roll, pitch, yaw, forceX, forceY, forceZ;

        // 데이터를 파싱
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw >> forceX >> forceY >> forceZ))
        {
            ROS_WARN("Failed to parse a line in the control source file. Skipping...");
            continue;
        }

        // Position 변환
        Vector4d position(x, y, z, 1.0);
        position = transformation * position;

        // Orientation 변환
        Matrix4d rotationMatrix = RPYToMatrix(roll, pitch, yaw);
        Matrix4d transformedRotation = transformation * rotationMatrix;
        Vector3d transformedRPY = MatrixToRPY(transformedRotation.block<3, 3>(0, 0));

        // 결과를 출력 파일에 저장
        control_outputFile << position(0) << " " << position(1) << " " << position(2) << " "
                           << transformedRPY(0) << " " << transformedRPY(1) << " " << transformedRPY(2) << " "
                           << forceX << " " << forceY << " " << forceZ << endl;
    }
    control_inputFile.close();
    control_outputFile.close();
    // 저장 완료 후 로깅
    std::cout << "waypoints_msg size: " << visual_waypoints_msg.waypoints.size() << std::endl;
    ROS_INFO("Transformation completed and saved to file!");

    // 퍼블리시 루프
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok())
    {
        waypoint_pub.publish(visual_waypoints_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
