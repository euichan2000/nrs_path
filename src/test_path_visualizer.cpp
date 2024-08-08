// 필요한 헤더 파일 포함
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/Console.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL_reader.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <iterator>
#include <std_msgs/String.h>

// CGAL 관련 타입 정의
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;
typedef Traits::Point_3 Point_3;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Triangle_mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Tree;

namespace ob = ompl::base;
namespace og = ompl::geometric;

// 전역 변수 선언
ros::Publisher waypoints_pub;
std::vector<geometry_msgs::Point> clicked_points;
Triangle_mesh tmesh;
Tree *tree;
nrs_vision_rviz::Waypoints waypoints_msg;
bool new_waypoints = false;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {}
    bool isValid(const ob::State *state) const override
    {
        return true;
    }
};

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "start")
    {
        waypoints_pub.publish(waypoints_msg);
        ROS_INFO("Waypoints published.");
    }
}

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    clicked_points.push_back(msg->point);
    new_waypoints = true; // 새로운 waypoint가 추가될 때 플래그를 true로 설정
}

bool locate_face_and_point(const Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location)
{
    if (tmesh.faces().empty())
    {
        ROS_ERROR("Mesh is empty, cannot build AABB tree.");
        return false;
    }

    Tree tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree.build();

    auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, tmesh);
    face = result.first;
    location = result.second;
    return true;
}

std::vector<geometry_msgs::Point> omplPlanAndSimplify(const std::vector<geometry_msgs::Point> &points)
{
    unsigned int maxSteps = 2;
    double minChange = std::numeric_limits<double>::epsilon();
    std::vector<geometry_msgs::Point> smooth_path;

    if (points.size() < 2)
        return smooth_path;

    auto state_space(std::make_shared<ob::RealVectorStateSpace>(3));
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    state_space->setBounds(bounds);

    og::SimpleSetup ss(state_space);
    ss.setStateValidityChecker(std::make_shared<ValidityChecker>(ss.getSpaceInformation()));

    og::PathGeometric full_path(ss.getSpaceInformation());

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        ob::ScopedState<> start(state_space), goal(state_space);

        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = points[i].x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = points[i].y;
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = points[i].z;

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = points[i + 1].x;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = points[i + 1].y;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = points[i + 1].z;

        ss.setStartAndGoalStates(start, goal);

        auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        ss.setPlanner(planner);

        ob::PlannerStatus solved = ss.solve(1.0);

        if (solved)
        {
            ss.simplifySolution();
            og::PathGeometric path = ss.getSolutionPath();

            if (i == 0)
            {
                full_path.append(path);
            }
            else
            {
                for (size_t j = 1; j < path.getStateCount(); ++j)
                {
                    full_path.append(path.getState(j));
                }
            }
        }
    }

    og::PathSimplifier simplifier(ss.getSpaceInformation());
    simplifier.smoothBSpline(full_path, maxSteps, minChange);

    for (size_t i = 0; i < full_path.getStateCount(); ++i)
    {
        const ob::RealVectorStateSpace::StateType *state = full_path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        geometry_msgs::Point p;
        p.x = state->values[0];
        p.y = state->values[1];
        p.z = state->values[2];
        smooth_path.push_back(p);
    }

    return smooth_path;
}

bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh)
{
    std::vector<Kernel::Point_3> points;
    std::vector<std::array<std::size_t, 3>> triangles;
    if (!CGAL::read_STL(input, points, triangles))
    {
        ROS_ERROR("Failed to read STL file.");
        return false;
    }

    std::map<std::size_t, vertex_descriptor> index_to_vertex;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        index_to_vertex[i] = mesh.add_vertex(points[i]);
    }

    for (const auto &t : triangles)
    {
        mesh.add_face(index_to_vertex[t[0]], index_to_vertex[t[1]], index_to_vertex[t[2]]);
    }
    ROS_INFO("Successfully read STL file.");
    return true;
}

std::vector<geometry_msgs::Point> projectPointsOntoMesh(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<geometry_msgs::Point> projected_points;

    for (const auto &point : points)
    {
        Point_3 query(point.x, point.y, point.z + 0.01);
        auto closest_point = tree->closest_point(query);

        geometry_msgs::Point ros_point;
        ros_point.x = closest_point.x();
        ros_point.y = closest_point.y();
        ros_point.z = closest_point.z();

        projected_points.push_back(ros_point);
    }

    return projected_points;
}

std::vector<nrs_vision_rviz::Waypoint> convertToWaypoints(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<nrs_vision_rviz::Waypoint> waypoints;

    for (const auto &point : points)
    {
        Point_3 cgal_point(point.x, point.y, point.z);
        face_descriptor face;
        Surface_mesh_shortest_path::Barycentric_coordinates location;

        if (!locate_face_and_point(cgal_point, face, location))
        {
            ROS_ERROR("Failed to locate face and point");
            continue;
        }

        Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, tmesh);

        nrs_vision_rviz::Waypoint waypoint_msg;
        waypoint_msg.point.x = point.x;
        waypoint_msg.point.y = point.y;
        waypoint_msg.point.z = point.z;

        waypoint_msg.normal.x = -normal.x();
        waypoint_msg.normal.y = -normal.y();
        waypoint_msg.normal.z = -normal.z();

        waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_path_visualizer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10);
    ros::Subscriber keyboard_sub = nh.subscribe("moveit_command", 10, keyboardCallback);

    std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/lid_wrap.stl";
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input || !read_stl_file(input, tmesh))
    {
        ROS_ERROR("Error: cannot read the file %s", mesh_file_path.c_str());
        return EXIT_FAILURE;
    }

    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();

    ros::Rate r(30);

    while (ros::ok())
    {
        ros::spinOnce();

        if (new_waypoints && clicked_points.size() > 1)
        {
            std::vector<geometry_msgs::Point> smooth_path = omplPlanAndSimplify(clicked_points);
            std::vector<geometry_msgs::Point> projected_path = projectPointsOntoMesh(smooth_path);
            waypoints_msg.waypoints = convertToWaypoints(projected_path); // waypoints_msg 업데이트

            // 원래 경로 시각화 (빨간색)
            visualization_msgs::Marker original_path_marker;
            original_path_marker.header.frame_id = "base_link";
            original_path_marker.header.stamp = ros::Time::now();
            original_path_marker.ns = "original_path";
            original_path_marker.id = 0;
            original_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            original_path_marker.action = visualization_msgs::Marker::ADD;
            original_path_marker.pose.orientation.w = 1.0;
            original_path_marker.scale.x = 0.005;
            original_path_marker.color.r = 1.0;
            original_path_marker.color.a = 1.0;

            for (const auto &point : smooth_path)
            {
                original_path_marker.points.push_back(point);
            }

            marker_pub.publish(original_path_marker);

            // 투영된 경로 시각화 (파란색)
            visualization_msgs::Marker projected_path_marker;
            projected_path_marker.header.frame_id = "base_link";
            projected_path_marker.header.stamp = ros::Time::now();
            projected_path_marker.ns = "projected_path";
            projected_path_marker.id = 1;
            projected_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            projected_path_marker.action = visualization_msgs::Marker::ADD;
            projected_path_marker.pose.orientation.w = 1.0;
            projected_path_marker.scale.x = 0.005;
            projected_path_marker.color.b = 1.0;
            projected_path_marker.color.a = 1.0;

            for (const auto &point : projected_path)
            {
                projected_path_marker.points.push_back(point);
            }

            marker_pub.publish(projected_path_marker);

            new_waypoints = false; // OMPL 플래닝이 끝난 후 플래그를 false로 설정
        }

        r.sleep();
    }

    delete tree;
    return 0;
}
