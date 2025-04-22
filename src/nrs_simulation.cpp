#include "nrs_simulation.h"
    // : move_group("manipulator")
// nrs_simulation::nrs_simulation()
// {
//     // 실제 초기화는 init() 함수에서 수행합니다.
// }

// nrs_simulation::~nrs_simulation()
// {
// }
bool nrs_simulation::initializeHomePosition()
{
    move_group.setPlanningTime(45.0);
    initial_pose = {
        {"shoulder_pan_joint", 10.95 * M_PI / 180},
        {"shoulder_lift_joint", -73.33 * M_PI / 180},
        {"elbow_joint", -116.49 * M_PI / 180},
        {"wrist_1_joint", -80.22 * M_PI / 180},
        {"wrist_2_joint", 89.60 * M_PI / 180},
        {"wrist_3_joint", 52.86 * M_PI / 180}};

    move_group.setJointValueTarget(initial_pose);
    move_group.move(); // 초기 pose로 이동 (blocking 호출)
    std::cout << "initial_pose_ok" << std::endl;
    return true;
}

// simulateFromWaypoints 함수: 주어진 웨이포인트들에 대해 툴팁 트랜스폼을 적용하고, MoveIt의 Cartesian path를 계산 및 실행한 후 초기 pose로 복귀합니다.
bool nrs_simulation::simulateFromWaypoints(const std::vector<geometry_msgs::Pose> &waypoints)
{

    // 툴팁 트랜스폼 설정 (예: z축 오프셋 -0.316 적용)
    tf2::Transform tooltip_transform;
    tooltip_transform.setOrigin(tf2::Vector3(0.0, 0.0, -0.316));

    tf2::Quaternion tooltip_quat;
    tooltip_quat.setRPY(0.0, 0.0, 0.0);

    tooltip_transform.setRotation(tooltip_quat);

    // 각 웨이포인트에 툴팁 트랜스폼 적용
    std::vector<geometry_msgs::Pose> transformed_waypoints;
    for (const auto &pose : waypoints)
    {
        tf2::Transform pose_tf;
        tf2::fromMsg(pose, pose_tf);
        tf2::Transform final_tf = pose_tf * tooltip_transform;
        geometry_msgs::Pose final_pose;
        final_pose.position.x = final_tf.getOrigin().x();
        final_pose.position.y = final_tf.getOrigin().y();
        final_pose.position.z = final_tf.getOrigin().z();
        final_pose.orientation = tf2::toMsg(final_tf.getRotation());
        transformed_waypoints.push_back(final_pose);
    }

    // MoveIt을 이용하여 Cartesian Path 계산
    moveit_msgs::RobotTrajectory trajectory_msg;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(transformed_waypoints, eef_step, jump_threshold, trajectory_msg);
    ROS_INFO("simulateFromWaypoints: Cartesian path computed (%.2f%% achieved)", fraction * 100.0);

    if (fraction < 1.0)
    {
        ROS_WARN("simulateFromWaypoints: Path planning incomplete (%.2f%% achieved)", fraction * 100.0);
    }

    // 계획 생성 및 실행
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_msg;
    // Create a RobotTrajectory object from the computed trajectory
    robot_trajectory::RobotTrajectory robot_trajectory(move_group.getCurrentState()->getRobotModel(), "manipulator");

    std::cout << "planning done" << std::endl;
    move_group.execute(plan);
    std::cout << "execute done" << std::endl;

    std::map<std::string, double> initial_pose = {
        {"shoulder_pan_joint", 10.95 * M_PI / 180},
        {"shoulder_lift_joint", -73.33 * M_PI / 180},
        {"elbow_joint", -116.49 * M_PI / 180},
        {"wrist_1_joint", -80.22 * M_PI / 180},
        {"wrist_2_joint", 89.60 * M_PI / 180},
        {"wrist_3_joint", 52.86 * M_PI / 180}};
    // 실행 후 초기 pose로 복귀
    move_group.setJointValueTarget(initial_pose);
    ROS_INFO("Starting move to initial pose...");
    move_group.move();
    ROS_INFO("Returned from move to initial pose.");

    ROS_INFO("simulateFromWaypoints: Simulation completed.");
    return true;
}
