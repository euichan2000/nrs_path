#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <nrs_vision_rviz/Waypoint.h>
#include <fstream> // 파일 입출력 라이브러리
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/String.h>

std::vector<geometry_msgs::Pose> waypoints_poses;
std::vector<geometry_msgs::Pose> interpolated_waypoints_poses;

bool start_planning = false;
bool using_interpolated_waypoints = false;

void keyboardCallback(const std_msgs::String::ConstPtr &msg);

geometry_msgs::Pose applyTooltipTransform(const geometry_msgs::Pose &pose, const tf2::Transform &tooltip_transform);

void saveJointStatesToFile(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &file_path);

void saveTCPStatesToFile(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &file_path, moveit::planning_interface::MoveGroupInterface &move_group);

void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg);

void interpolatedWaypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_planner");
    ros::NodeHandle nh;

    ros::Subscriber waypoints_sub = nh.subscribe("waypoints_with_normals", 10, waypointsCallback);
    //ros::Subscriber interpolated_waypoints_sub = nh.subscribe("interpolated_waypoints_with_normals", 10, interpolatedWaypointsCallback);
    ros::Subscriber keyboard_sub = nh.subscribe("moveit_command", 10, keyboardCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(45.0);

    std::map<std::string, double> initial_pose = {
        {"shoulder_pan_joint", 10.71 * M_PI / 180},
        {"shoulder_lift_joint", -55.43 * M_PI / 180},
        {"elbow_joint", -135.87 * M_PI / 180},
        {"wrist_1_joint", -74.46 * M_PI / 180},
        {"wrist_2_joint", 88.39 * M_PI / 180},
        {"wrist_3_joint", 0.58 * M_PI / 180}};

    move_group.setJointValueTarget(initial_pose);
    move_group.move();

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    while (ros::ok())
    {
        if (start_planning)
        {
            std::vector<geometry_msgs::Pose> current_waypoints;

            if (using_interpolated_waypoints)
            {
                current_waypoints = interpolated_waypoints_poses;
            }
            else
            {
                current_waypoints = waypoints_poses;
            }

            if (current_waypoints.size() > 1)
            {
                move_group.clearPathConstraints();
                moveit_msgs::RobotTrajectory trajectory_msg;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                double fraction = move_group.computeCartesianPath(current_waypoints, eef_step, jump_threshold, trajectory_msg);

                ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

                if (fraction > 0.0)
                {
                    // Create a plan
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory_msg;

                    // Create a RobotTrajectory object from the computed trajectory
                    robot_trajectory::RobotTrajectory robot_trajectory(move_group.getCurrentState()->getRobotModel(), "manipulator");

                    // Convert the moveit_msgs::RobotTrajectory to robot_trajectory::RobotTrajectory
                    robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

                    // Apply time parameterization to ensure constant speed
                    trajectory_processing::IterativeSplineParameterization time_param;
                    bool success = time_param.computeTimeStamps(robot_trajectory);

                    if (!success)
                    {
                        ROS_WARN("Time parameterization failed");
                    }

                    // Convert the robot_trajectory back to a moveit_msgs::RobotTrajectory
                    robot_trajectory.getRobotTrajectoryMsg(plan.trajectory_);

                    move_group.execute(plan);
                    move_group.setJointValueTarget(initial_pose);
                    move_group.move();

                    display_trajectory.trajectory_start = plan.start_state_;
                    display_trajectory.trajectory.clear();
                    display_trajectory.trajectory.push_back(plan.trajectory_);
                    display_publisher.publish(display_trajectory);

                    // std::string file_path = using_interpolated_waypoints ? "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/interpolated_joint_states.txt" : "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/joint_states.txt";
                    // saveJointStatesToFile(plan, file_path);

                    // std::string file_path2 = "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/tcp_states.txt";
                    // saveTCPStatesToFile(plan, file_path2, move_group);
                }
            }
            start_planning = false;
        }

        ros::Duration(1.0).sleep();
    }

    ros::shutdown();
    return 0;
}

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "waypoints")
    {
        start_planning = true;
        using_interpolated_waypoints = false;
        ROS_INFO("Motion planning using waypoints_with_normals");
    }
    else if (msg->data == "interpolated_waypoints")
    {
        start_planning = true;
        using_interpolated_waypoints = true;
        ROS_INFO("Motion planning using interpolated_waypoints_with_normals");
    }

    else if (msg->data == "reset")
    {

        waypoints_poses.clear();
        interpolated_waypoints_poses.clear();

        ROS_INFO("waypoints cleared");
    }
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

void saveTCPStatesToFile(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &file_path, moveit::planning_interface::MoveGroupInterface &move_group)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open TCP states file for writing");
        return;
    }

    const trajectory_msgs::JointTrajectory &trajectory = plan.trajectory_.joint_trajectory;
    const std::string &end_effector_link = move_group.getEndEffectorLink();
    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());

    // Iterate through the trajectory points
    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        // Update robot state to current trajectory point
        moveit::core::RobotState robot_state = *move_group.getCurrentState();
        robot_state.setVariablePositions(trajectory.points[i].positions);
        robot_state.updateLinkTransforms();

        // Extract TCP position from the end effector state
        const Eigen::Affine3d &end_effector_state = robot_state.getGlobalLinkTransform(end_effector_link);
        Eigen::Vector3d tcp_position = end_effector_state.translation();

        file << "Step " << i << ":\n";
        file << "TCP Position:\n";
        file << "x: " << tcp_position.x() << "\n";
        file << "y: " << tcp_position.y() << "\n";
        file << "z: " << tcp_position.z() << "\n";

        // Calculate TCP velocity if velocities are available
        if (!trajectory.points[i].velocities.empty())
        {
            Eigen::MatrixXd jacobian;
            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); // Reference point at TCP

            // Compute the Jacobian for the end effector link
            robot_state.getJacobian(joint_model_group, robot_state.getLinkModel(end_effector_link), reference_point_position, jacobian);

            // Convert joint velocities to Eigen vector
            Eigen::VectorXd joint_velocities = Eigen::VectorXd::Map(trajectory.points[i].velocities.data(), trajectory.points[i].velocities.size());

            // Compute TCP velocity in world frame
            Eigen::Vector3d tcp_velocity = jacobian.block(0, 0, 3, joint_velocities.size()) * joint_velocities;

            file << "TCP Velocity:\n";
            file << "vx: " << tcp_velocity.x() << "\n";
            file << "vy: " << tcp_velocity.y() << "\n";
            file << "vz: " << tcp_velocity.z() << "\n";
        }

        // Calculate TCP acceleration if accelerations are available
        if (!trajectory.points[i].accelerations.empty())
        {
            Eigen::MatrixXd jacobian;
            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); // Reference point at TCP

            // Compute the Jacobian for the end effector link
            robot_state.getJacobian(joint_model_group, robot_state.getLinkModel(end_effector_link), reference_point_position, jacobian);

            // Convert joint accelerations to Eigen vector
            Eigen::VectorXd joint_accelerations = Eigen::VectorXd::Map(trajectory.points[i].accelerations.data(), trajectory.points[i].accelerations.size());

            // Compute TCP acceleration in world frame
            Eigen::Vector3d tcp_acceleration = jacobian.block(0, 0, 3, joint_accelerations.size()) * joint_accelerations;

            file << "TCP Acceleration:\n";
            file << "ax: " << tcp_acceleration.x() << "\n";
            file << "ay: " << tcp_acceleration.y() << "\n";
            file << "az: " << tcp_acceleration.z() << "\n";
        }

        file << "----------\n";
    }

    file.close();
    ROS_INFO("TCP states saved to %s", file_path.c_str());
}

void saveJointStatesToFile(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &file_path)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open joint states file for writing");
        return;
    }

    const trajectory_msgs::JointTrajectory &trajectory = plan.trajectory_.joint_trajectory;

    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        const trajectory_msgs::JointTrajectoryPoint &point = trajectory.points[i];
        file << "Step " << i << ":\n";

        file << "Positions:\n";
        for (size_t j = 0; j < point.positions.size(); ++j)
        {
            file << trajectory.joint_names[j] << ": " << point.positions[j] << "\n";
        }

        file << "Velocities:\n";
        for (size_t j = 0; j < point.velocities.size(); ++j)
        {
            file << trajectory.joint_names[j] << ": " << point.velocities[j] << "\n";
        }

        file << "Accelerations:\n";
        for (size_t j = 0; j < point.accelerations.size(); ++j)
        {
            file << trajectory.joint_names[j] << ": " << point.accelerations[j] << "\n";
        }

        file << "----------\n";
    }

    file.close();
    ROS_INFO("Joint states saved to %s", file_path.c_str());
}

void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{
    waypoints_poses.clear();

    tf2::Transform tooltip_transform;
    tooltip_transform.setOrigin(tf2::Vector3(0.0, 0.0, -0.316));
    tf2::Quaternion tooltip_orientation;
    tooltip_orientation.setRPY(0.0, 0.0, 0.0);
    tooltip_transform.setRotation(tooltip_orientation);

    for (const auto &waypoint : msg->waypoints)
    {
        geometry_msgs::Pose target_pose;
        target_pose.position = waypoint.point;

        tf2::Vector3 z_axis(waypoint.normal.x, waypoint.normal.y, waypoint.normal.z);
        tf2::Vector3 x_axis(1, 0, 0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();

        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);
        target_pose.orientation = tf2::toMsg(orientation);

        geometry_msgs::Pose final_pose = applyTooltipTransform(target_pose, tooltip_transform);
        waypoints_poses.push_back(final_pose);
    }
}

void interpolatedWaypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{
    interpolated_waypoints_poses.clear();

    tf2::Transform tooltip_transform;
    tooltip_transform.setOrigin(tf2::Vector3(0.0, 0.0, -0.316));
    tf2::Quaternion tooltip_orientation;
    tooltip_orientation.setRPY(0.0, 0.0, 0.0);
    tooltip_transform.setRotation(tooltip_orientation);

    for (const auto &waypoint : msg->waypoints)
    {
        geometry_msgs::Pose target_pose;
        target_pose.position = waypoint.point;

        tf2::Vector3 z_axis(waypoint.normal.x, waypoint.normal.y, waypoint.normal.z);
        tf2::Vector3 x_axis(1, 0, 0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();

        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);
        target_pose.orientation = tf2::toMsg(orientation);

        geometry_msgs::Pose final_pose = applyTooltipTransform(target_pose, tooltip_transform);
        interpolated_waypoints_poses.push_back(final_pose);
    }
}
