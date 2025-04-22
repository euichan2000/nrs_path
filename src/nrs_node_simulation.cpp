#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <iostream>

// 전역 변수로 사용할 MoveGroupInterface 포인터와 초기 pose, 텍스트 파일 경로

std::map<std::string, double> initial_pose;
std::string interpolated_waypoints_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/final_waypoints.txt";
std::vector<geometry_msgs::Pose> final_waypoints;
bool start_planning = false;

std::vector<geometry_msgs::Pose> applyTooltipTransform(const std::vector<geometry_msgs::Pose> &poses, const tf2::Transform &tooltip_transform)
{
    std::vector<geometry_msgs::Pose> transformed_poses;
    transformed_poses.reserve(poses.size());

    for (const auto &pose : poses)
    {
        tf2::Transform waypoint_transform;
        tf2::fromMsg(pose, waypoint_transform);
        tf2::Transform final_transform = waypoint_transform * tooltip_transform;

        geometry_msgs::Pose final_pose;
        final_pose.position.x = final_transform.getOrigin().x();
        final_pose.position.y = final_transform.getOrigin().y();
        final_pose.position.z = final_transform.getOrigin().z();
        final_pose.orientation = tf2::toMsg(final_transform.getRotation());

        transformed_poses.push_back(final_pose);
    }
    return transformed_poses;
}

// moveitSimulationCallback 서비스 콜백 함수: 텍스트 파일에서 웨이포인트를 읽어 simulateFromWaypoints() 함수를 호출
bool moveitSimulationCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    std::ifstream file(interpolated_waypoints_file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", interpolated_waypoints_file_path.c_str());
        return false;
    }

    std::vector<geometry_msgs::Pose> waypoints;
    std::string line;
    int line_number = 0;
    while (std::getline(file, line))
    {
        line_number++; // 각 줄마다 번호 증가
        if (line.empty())
            continue;
        // 예시: 100줄에 한 번씩 샘플링 (필요에 따라 조정)
        if (line_number % 100 != 0)
            continue;

        std::istringstream iss(line);
        double x, y, z, roll, pitch, yaw;
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw))
        {
            ROS_WARN("Failed to parse line: %s", line.c_str());
            continue;
        }
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        pose.orientation = tf2::toMsg(quat);

        waypoints.push_back(pose);
    }
    file.close();

    tf2::Transform tooltip_transform;
    tooltip_transform.setOrigin(tf2::Vector3(0.0, 0.0, -0.31));
    tf2::Quaternion tooltip_orientation;
    tooltip_orientation.setRPY(0.0, 0.0, 0.0);
    tooltip_transform.setRotation(tooltip_orientation);

    final_waypoints = applyTooltipTransform(waypoints, tooltip_transform);
    start_planning = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_node_simulation");
    ros::NodeHandle nh;

    // simulation 서비스 등록
    ros::ServiceServer simulation_service = nh.advertiseService("simulation", moveitSimulationCallback);
    ROS_INFO("nrs_node_simulation started. Waiting for simulation service call...");

    // AsyncSpinner를 통해 콜백 처리를 백그라운드 스레드에서 수행합니다.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(45.0);

    std::map<std::string, double> initial_pose = {
        {"shoulder_pan_joint", 10.95 * M_PI / 180},
        {"shoulder_lift_joint", -73.33 * M_PI / 180},
        {"elbow_joint", -116.49 * M_PI / 180},
        {"wrist_1_joint", -80.22 * M_PI / 180},
        {"wrist_2_joint", 89.60 * M_PI / 180},
        {"wrist_3_joint", 52.86 * M_PI / 180}};

    move_group.setJointValueTarget(initial_pose);
    move_group.move();
    std::cout << "initial_pose done" << std::endl;

    while (ros::ok())
    {
        if (start_planning)
        {

            if (final_waypoints.size() > 1)
            {

                move_group.clearPathConstraints();
                moveit_msgs::RobotTrajectory trajectory_msg;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                double fraction = move_group.computeCartesianPath(final_waypoints, eef_step, jump_threshold, trajectory_msg);

                ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

                if (fraction > 0.0)
                {
                    // Create a plan
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory_msg;

                    // Create a RobotTrajectory object from the computed trajectory
                    robot_trajectory::RobotTrajectory robot_trajectory(move_group.getCurrentState()->getRobotModel(), "manipulator");

                    move_group.execute(plan);
                    move_group.setJointValueTarget(initial_pose);
                    move_group.move();
                }
            }
            start_planning = false;
        }

        ros::Duration(1.0).sleep();
    }

    ros::shutdown();

    return 0;
}
