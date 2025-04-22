#ifndef NRS_SIMULATION_H
#define NRS_SIMULATION_H

#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <fstream>
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <map>
#include <string>

/**
 * @brief nrs_simulation 클래스는 txt 파일로부터 읽어온 웨이포인트 데이터를 기반으로 MoveIt 시뮬레이션을 수행합니다.
 *
 * 이 클래스는 주어진 웨이포인트에 대해 툴팁 트랜스폼을 적용하고,
 * Cartesian 경로를 계산하여 실행한 후, 시뮬레이션이 끝나면 초기 pose로 복귀합니다.
 */
class nrs_simulation
{
public:
    // nrs_simulation();
    // ~nrs_simulation();
    bool initializeHomePosition();
    bool simulateFromWaypoints(const std::vector<geometry_msgs::Pose> &waypoints);

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    // 초기 관절값 (초기 pose로 복귀하기 위한 값)
    std::map<std::string, double> initial_pose;
};

#endif // NRS_SIMULATION_H
