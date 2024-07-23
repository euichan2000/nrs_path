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

std::vector<geometry_msgs::Pose> waypoints_poses;
bool path_received = false;

void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg)
{
    waypoints_poses.clear();
    for (const auto &waypoint : msg->waypoints)
    {
        geometry_msgs::Pose target_pose;
        target_pose.position = waypoint.point;

        // End effector가 항상 표면에 수직하도록 orientation 설정
        tf2::Vector3 z_axis(waypoint.normal.x, waypoint.normal.y, waypoint.normal.z);
        tf2::Vector3 x_axis(1, 0, 0); // 기본 x 방향
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();

        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);
        target_pose.orientation = tf2::toMsg(orientation);
        waypoints_poses.push_back(target_pose);
        // ROS_INFO("waypoints_poses Size: %zu", waypoints_poses.size());
    }
    path_received = true;
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "mesh_object";

    // Define a pose for the mesh (assuming it's located at (0, 0, 0) with no rotation)
    geometry_msgs::Pose mesh_pose;
    mesh_pose.position.x = 0.0;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = 0.0;
    mesh_pose.orientation.w = 1.0;

    // Load mesh from file
    shapes::Mesh *m = shapes::createMeshFromResource("package://nrs_vision_rviz/mesh/lid_wrap.stl");
    shape_msgs::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_shape;
    shapes::constructMsgFromShape(m, mesh_msg_shape);
    mesh_msg = boost::get<shape_msgs::Mesh>(mesh_msg_shape);

    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_planner");
    ros::NodeHandle nh;
    ros::Subscriber waypoints_sub = nh.subscribe("waypoints_with_normals", 10, waypointsCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(45.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObject(planning_scene_interface);

    // RViz에서 Trajectory를 시각화하기 위해 디스플레이 트래젝트리 퍼블리셔 설정
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    while (ros::ok())
    {
        if (path_received && waypoints_poses.size() > 1)
        {
            // 기존 경로를 삭제
            move_group.clearPathConstraints();
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints_poses, eef_step, jump_threshold, trajectory);

            ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

            // 계획된 경로를 실행 및 시각화
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);

            display_trajectory.trajectory_start = plan.start_state_;
            display_trajectory.trajectory.clear(); // 이전의 trajectory를 비웁니다.
            display_trajectory.trajectory.push_back(plan.trajectory_);
            // display_publisher.publish(display_trajectory);

            waypoints_poses.clear();
            path_received = false;
        }

        ros::Duration(1.0).sleep();
    }

    ros::shutdown();
    return 0;
}
