#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <nrs_vision_rviz/Waypoint.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <fstream> // 파일 입출력 라이브러리
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/String.h>

std::vector<geometry_msgs::Pose> waypoints_poses;
bool path_received = false;
bool start_planning = false; // 플래닝 시작 플래그

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "start")
    {
        start_planning = true;
        ROS_INFO("Received start command, initiating planning.");
    }
}

void saveWaypointsToFile(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{
    std::ofstream file("/home/nrs/catkin_ws/src/nrs_vision_rviz/waypoints/waypoints.txt");
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open waypoints file for writing");
        return;
    }

    for (const auto &waypoint : msg->waypoints)
    {
        file << waypoint.point.x << " "
             << waypoint.point.y << " "
             << waypoint.point.z << " "
             << waypoint.normal.x << " "
             << waypoint.normal.y << " "
             << waypoint.normal.z << std::endl;
    }

    file.close();
    ROS_INFO("Waypoints saved to waypoints.txt");
}

geometry_msgs::Pose applyTooltipTransform(const geometry_msgs::Pose &pose, const tf2::Transform &tooltip_transform)
{
    tf2::Transform waypoint_transform;
    tf2::fromMsg(pose, waypoint_transform);
    tf2::Transform final_transform = waypoint_transform * tooltip_transform;

    geometry_msgs::Pose final_pose;
    final_pose.position.x = final_transform.getOrigin().x();
    final_pose.position.y = final_transform.getOrigin().y();
    final_pose.position.z = final_transform.getOrigin().z();
    final_pose.orientation = tf2::toMsg(final_transform.getRotation());
    return final_pose;
}

void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{
    waypoints_poses.clear();

    // 툴팁 변환 정의 (wrist_3_link에 상대적인 위치 및 방향)
    tf2::Transform tooltip_transform;
    tooltip_transform.setOrigin(tf2::Vector3(-0.03, 0.0, -0.115)); // 툴팁의 위치 (필요시 변경)
    tf2::Quaternion tooltip_orientation;
    tooltip_orientation.setRPY(0.0, 0.0, M_PI_2); // 툴팁의 회전 (필요시 변경)
    tooltip_transform.setRotation(tooltip_orientation);

    for (size_t i = 0; i < msg->waypoints.size(); ++i)
    {
        geometry_msgs::Pose target_pose;
        target_pose.position = msg->waypoints[i].point;

        // z_axis는 웨이포인트의 표면 법선 벡터입니다.
        tf2::Vector3 z_axis(msg->waypoints[i].normal.x, msg->waypoints[i].normal.y, msg->waypoints[i].normal.z);
        // x_axis는 경로의 진행 방향을 나타냅니다.
        tf2::Vector3 x_axis(1, 0, 0); // 기본 x 방향
        // y_axis는 x_axis와 z_axis의 외적입니다.
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();

        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);
        target_pose.orientation = tf2::toMsg(orientation);

        // 툴팁 변환을 적용하여 최종 목표 자세를 계산
        geometry_msgs::Pose final_pose = applyTooltipTransform(target_pose, tooltip_transform);
        waypoints_poses.push_back(final_pose);
    }
    path_received = true;

    saveWaypointsToFile(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_planner");
    ros::NodeHandle nh;
    ros::Subscriber waypoints_sub = nh.subscribe("waypoints_with_normals", 10, waypointsCallback);
     ros::Subscriber keyboard_sub = nh.subscribe("moveit_command", 10, keyboardCallback); // "moveit_command" 구독 추가
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(45.0);

    // 초기 위치 설정
    std::map<std::string, double> initial_pose, home_pose;
    initial_pose["shoulder_pan_joint"] = 10.71 * M_PI / 180;
    initial_pose["shoulder_lift_joint"] = -55.43 * M_PI / 180;
    initial_pose["elbow_joint"] = -135.87 * M_PI / 180;
    initial_pose["wrist_1_joint"] = -74.46 * M_PI / 180;
    initial_pose["wrist_2_joint"] = 88.39 * M_PI / 180;
    initial_pose["wrist_3_joint"] = 0.58 * M_PI / 180;

    home_pose["shoulder_pan_joint"] = 10.71 * M_PI / 180;
    home_pose["shoulder_lift_joint"] = -90.43 * M_PI / 180;
    home_pose["elbow_joint"] = 0.0 * M_PI / 180;
    home_pose["wrist_1_joint"] = -74.46 * M_PI / 180;
    home_pose["wrist_2_joint"] = 88.39 * M_PI / 180;
    home_pose["wrist_3_joint"] = 0.58 * M_PI / 180;

    move_group.setJointValueTarget(initial_pose);
    move_group.move();

    // RViz에서 Trajectory를 시각화하기 위해 디스플레이 트래젝트리 퍼블리셔 설정
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    while (ros::ok())
    {
        if (path_received && waypoints_poses.size() > 1 && start_planning)
        {
            // 기존 경로를 삭제
            move_group.clearPathConstraints();
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints_poses, eef_step, jump_threshold, trajectory);

            ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

            // 계획된 경로를 실행 및 시각화
            if (fraction > 0.0)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;

                // Check for collisions

                move_group.execute(plan);
                move_group.setJointValueTarget(initial_pose);
                move_group.move();

                display_trajectory.trajectory_start = plan.start_state_;
                display_trajectory.trajectory.clear(); // 이전의 trajectory를 비웁니다.
                display_trajectory.trajectory.push_back(plan.trajectory_);
                display_publisher.publish(display_trajectory);
            }

            waypoints_poses.clear();
            path_received = false;
            start_planning = false; // 플래닝이 완료되면 다시 대기 상태로 설정
        }

        ros::Duration(1.0).sleep();
    }

    ros::shutdown();
    return 0;
}
