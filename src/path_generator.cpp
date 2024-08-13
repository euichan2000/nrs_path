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
ros::Publisher marker_pub;
std::vector<geometry_msgs::Point> clicked_points;
Triangle_mesh tmesh;
Tree *tree;
Surface_mesh_shortest_path *shortest_paths;
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

void visualizePath(const std::vector<geometry_msgs::Point> &path, const std::string &ns, int id, float r, float g, float b, float a)
{
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "base_link";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = ns;
    path_marker.id = id;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.005;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;
    path_marker.color.a = a;

    for (const auto &point : path)
    {
        path_marker.points.push_back(point);
    }

    marker_pub.publish(path_marker);
}

void visualizeWaypointsAxes(const std::vector<nrs_vision_rviz::Waypoint> &waypoints)
{
    double axis_length = 0.01; // 축의 고정 길이

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        const auto &waypoint = waypoints[i];
        const auto &origin = waypoint.point;
        const auto &z_axis = waypoint.normal;

        // 다음 점을 기준으로 x축 방향을 설정 (path 진행 방향)
        geometry_msgs::Point x_axis_end;
        if (i < waypoints.size() - 1)
        {
            const auto &next_point = waypoints[i + 1].point;

            // x축 벡터 계산
            geometry_msgs::Point x_axis;
            x_axis.x = next_point.x - origin.x;
            x_axis.y = next_point.y - origin.y;
            x_axis.z = next_point.z - origin.z;

            // 벡터를 정규화
            double length = std::sqrt(x_axis.x * x_axis.x + x_axis.y * x_axis.y + x_axis.z * x_axis.z);
            x_axis.x = (x_axis.x / length) * axis_length;
            x_axis.y = (x_axis.y / length) * axis_length;
            x_axis.z = (x_axis.z / length) * axis_length;

            // x_axis_end 포인트 계산
            x_axis_end.x = origin.x + x_axis.x;
            x_axis_end.y = origin.y + x_axis.y;
            x_axis_end.z = origin.z + x_axis.z;
        }
        else
        {
            // 마지막 점인 경우, x축 방향을 0으로 설정
            x_axis_end = origin;
        }

        // y축을 z축과 x축의 외적(cross product)으로 계산
        geometry_msgs::Point y_axis_end;
        y_axis_end.x = origin.x + (-z_axis.y * (x_axis_end.z - origin.z) - -z_axis.z * (x_axis_end.y - origin.y));
        y_axis_end.y = origin.y + (-z_axis.z * (x_axis_end.x - origin.x) - -z_axis.x * (x_axis_end.z - origin.z));
        y_axis_end.z = origin.z + (-z_axis.x * (x_axis_end.y - origin.y) - -z_axis.y * (x_axis_end.x - origin.x));

        // z축 끝 포인트 계산 (정규화)
        geometry_msgs::Point z_axis_end;
        double z_length = std::sqrt(-z_axis.x * -z_axis.x + -z_axis.y * -z_axis.y + -z_axis.z * -z_axis.z);
        z_axis_end.x = origin.x + (-z_axis.x / z_length) * axis_length;
        z_axis_end.y = origin.y + (-z_axis.y / z_length) * axis_length;
        z_axis_end.z = origin.z + (-z_axis.z / z_length) * axis_length;

        // 마커 생성
        visualization_msgs::Marker x_axis_marker, y_axis_marker, z_axis_marker;

        x_axis_marker.header.frame_id = y_axis_marker.header.frame_id = z_axis_marker.header.frame_id = "base_link";
        x_axis_marker.header.stamp = y_axis_marker.header.stamp = z_axis_marker.header.stamp = ros::Time::now();
        x_axis_marker.ns = y_axis_marker.ns = z_axis_marker.ns = "waypoint_axes";
        x_axis_marker.action = y_axis_marker.action = z_axis_marker.action = visualization_msgs::Marker::ADD;
        x_axis_marker.type = y_axis_marker.type = z_axis_marker.type = visualization_msgs::Marker::ARROW;
        x_axis_marker.scale.x = y_axis_marker.scale.x = z_axis_marker.scale.x = 0.0005; // 화살표 몸통의 두께
        x_axis_marker.scale.y = y_axis_marker.scale.y = z_axis_marker.scale.y = 0.001;  // 화살표 머리의 두께
        x_axis_marker.scale.z = y_axis_marker.scale.z = z_axis_marker.scale.z = 0.001;  // 화살표 머리의 길이
        x_axis_marker.color.a = y_axis_marker.color.a = z_axis_marker.color.a = 1.0;

        // x축 색상 (빨간색)
        x_axis_marker.color.r = 1.0;
        x_axis_marker.color.g = 0.0;
        x_axis_marker.color.b = 0.0;

        // y축 색상 (녹색)
        y_axis_marker.color.r = 0.0;
        y_axis_marker.color.g = 1.0;
        y_axis_marker.color.b = 0.0;

        // z축 색상 (파란색)
        z_axis_marker.color.r = 0.0;
        z_axis_marker.color.g = 0.0;
        z_axis_marker.color.b = 1.0;

        // 각 마커의 고유 ID 설정
        x_axis_marker.id = i * 3;
        y_axis_marker.id = i * 3 + 1;
        z_axis_marker.id = i * 3 + 2;

        // 마커 시작점과 끝점 설정
        x_axis_marker.points.push_back(origin);
        x_axis_marker.points.push_back(x_axis_end);

        y_axis_marker.points.push_back(origin);
        y_axis_marker.points.push_back(y_axis_end);

        z_axis_marker.points.push_back(origin);
        z_axis_marker.points.push_back(z_axis_end);

        // 마커 퍼블리시
        marker_pub.publish(x_axis_marker);
        marker_pub.publish(y_axis_marker);
        marker_pub.publish(z_axis_marker);
    }
}

std::vector<geometry_msgs::Point> projectPointsOntoMesh(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<geometry_msgs::Point> projected_points, offset_points;

    for (const auto &point : points)
    {
        Point_3 query(point.x, point.y, point.z);

        // 해당 포인트에 가장 가까운 face와 위치를 찾습니다.
        face_descriptor face;
        Surface_mesh_shortest_path::Barycentric_coordinates location;
        if (!locate_face_and_point(query, face, location))
        {
            ROS_ERROR("Failed to locate face and point");
            continue;
        }

        // face의 normal vector를 계산합니다.
        Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, tmesh);

        // normal vector 방향으로 포인트를 살짝 이동시킵니다.
        double offset = 0.1; // 이동할 거리
        Point_3 offset_point = query + normal * offset;
        geometry_msgs::Point off_point;
        off_point.x = offset_point.x();
        off_point.y = offset_point.y();
        off_point.z = offset_point.z();
        offset_points.push_back(off_point);

        // mesh 표면에 가장 가까운 점을 찾습니다.
        auto closest_point = tree->closest_point(offset_point);

        geometry_msgs::Point ros_point;
        ros_point.x = closest_point.x();
        ros_point.y = closest_point.y();
        ros_point.z = closest_point.z();

        projected_points.push_back(ros_point);
    }
    visualizePath(offset_points, "offset_point", 6, 0.0, 0.0, 1.0, 1.0);

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

void generate_Geodesic_Path(const std::vector<geometry_msgs::Point> &points)
{

    std::vector<Point_3> complete_path;
    std::vector<Point_3> path_segment;

    for (size_t i = 1; i < points.size(); ++i)
    {
        Point_3 start(points[i - 1].x, points[i - 1].y, points[i - 1].z);
        Point_3 end(points[i].x, points[i].y, points[i].z);

        face_descriptor start_face, end_face;
        Surface_mesh_shortest_path::Barycentric_coordinates start_location, end_location;

        locate_face_and_point(start, start_face, start_location);
        locate_face_and_point(end, end_face, end_location);
        shortest_paths->add_source_point(end_face, end_location);
        shortest_paths->shortest_path_points_to_source_points(start_face, start_location, std::back_inserter(path_segment));
        shortest_paths->remove_all_source_points();
        complete_path.clear();
        complete_path.insert(complete_path.end(), path_segment.begin(), path_segment.end());
        shortest_paths->remove_all_source_points();
    }

    std::vector<geometry_msgs::Point> path_points;
    for (const auto &point : complete_path)
    {
        geometry_msgs::Point ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_point.z = point.z();
        path_points.push_back(ros_point);
    }

    ROS_INFO("Generated geodesic path with %zu points", path_points.size());
    visualizePath(path_points, "geodesic_path", 0, 0.0, 1.0, 0.0, 1.0);
    waypoints_msg.waypoints = convertToWaypoints(path_points);
    // visualizeWaypointsAxes(waypoints_msg.waypoints);
}

void generate_B_Splin_Path(const std::vector<geometry_msgs::Point> &points)
{
    unsigned int maxSteps = 2;
    double minChange = std::numeric_limits<double>::epsilon();
    std::vector<geometry_msgs::Point> smooth_path;

    if (points.size() < 2)
        return;

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
    std::vector<geometry_msgs::Point> projected_path = projectPointsOntoMesh(smooth_path);
    visualizePath(projected_path, "B_spline_path", 1, 0.0, 1.0, 0.0, 1.0);
    waypoints_msg.waypoints = convertToWaypoints(projected_path);
}
void generate_Catmull_Rom_Path(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<geometry_msgs::Point> smooth_path;
    if (points.size() < 4)
        return; // Need at least 4 points for Catmull-Rom spline

    for (size_t i = 1; i < points.size() - 2; ++i)
    {
        geometry_msgs::Point p0 = points[i - 1];
        geometry_msgs::Point p1 = points[i];
        geometry_msgs::Point p2 = points[i + 1];
        geometry_msgs::Point p3 = points[i + 2];

        for (double t = 0; t <= 1; t += 0.05)
        {
            double t2 = t * t;
            double t3 = t2 * t;

            geometry_msgs::Point p;
            p.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
            p.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
            p.z = 0.5 * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 + (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3);

            smooth_path.push_back(p);
        }
    }

    std::vector<geometry_msgs::Point> projected_path = projectPointsOntoMesh(smooth_path);
    visualizePath(projected_path, "Catmull_Rom_path", 2, 0.0, 0.0, 1.0, 1.0);
    waypoints_msg.waypoints = convertToWaypoints(projected_path);
    visualizeWaypointsAxes(waypoints_msg.waypoints);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10);
    ros::Subscriber keyboard_sub = nh.subscribe("moveit_command", 10, keyboardCallback);

    std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/lid_wrap.stl";
    std::ifstream input(mesh_file_path, std::ios::binary);
    read_stl_file(input, tmesh);

    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(tmesh);

    ros::Rate r(30);

    while (ros::ok())
    {
        ros::spinOnce();
        if (new_waypoints && clicked_points.size() > 1)
        {
            generate_B_Splin_Path(clicked_points);
            new_waypoints = false; // 모션 플래닝이 끝난 후 플래그를 false로 설정
        }

        r.sleep();
    }

    delete tree;
    delete shortest_paths;
    return 0;
}
