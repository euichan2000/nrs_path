// path_generator.cpp

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <limits>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
ros::Publisher controlpoints_pub;

Triangle_mesh tmesh;
Tree *tree;
Surface_mesh_shortest_path *shortest_paths;
nrs_vision_rviz::Waypoints waypoints_msg;
std_msgs::Float64MultiArray control_points_msg;

std::vector<geometry_msgs::Point> clicked_points;
std::vector<Eigen::Vector3d> selected_points; // Projected [clicked_points] onto Mesh surface
std::vector<double> u_values = {0.0};         // Interpolation Paramter array. U0=0
std::vector<Eigen::Vector3d> tangent_vectors; // Tangent Vectors array
std::vector<std::vector<Eigen::Vector3d>> bezier_control_points;
int steps;
double waypoints_distance = 0.0;

bool new_waypoints = false;
bool start_path_generating = false; // keyboard publisher order to start
bool use_straight = false;
bool use_spline = false;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {}
    bool isValid(const ob::State *state) const override
    {
        return true;
    }
};

// Structure for basic calculation of Triangle vertices
struct Vec3d
{
    double x, y, z;

    Vec3d() : x(0), y(0), z(0) {}

    Vec3d(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3d operator-(const Vec3d &other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3d operator+(const Vec3d &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3d operator*(double scalar) const
    {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3d cross(const Vec3d &other) const
    {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x};
    }

    double dot(const Vec3d &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    double length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3d normalize() const
    {
        double len = length();
        if (len > 0)
        {
            return {x / len, y / len, z / len};
        }
        return {0, 0, 0};
    }
};

// Triangle bertices & Normal
struct TriangleFace
{
    Vec3d vertices[3];
    Vec3d normal;
};

// double * Vec3d
Vec3d operator*(double scalar, const Vec3d &vec);
// Vec3d to Eigen::Vector3d
Vec3d EigenToVec3d(const Eigen::Vector3d &eigen_vec);
// Eigen::Vector3d to Vec3d
Eigen::Vector3d Vec3dToEigen(const Vec3d &vec);
// Kernel::Point_3 to Vec3d
Vec3d CGALPointToVec3d(const Point_3 &p);
// Vec3d to Kernel::Point_3
Point_3 Vec3dToCGALPoint(const Vec3d &v);

std::vector<geometry_msgs::Point> projectPointsOntoMesh(const std::vector<geometry_msgs::Point> &points);
// tranform triangle to basic calculation
std::vector<TriangleFace> convertMeshToTriangleFaces(const Triangle_mesh &tmesh);
// Find closest [face and barycentric coordinate on Mesh] with points
bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &tmesh);
// points to waypoints to publish
nrs_vision_rviz::Waypoints convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh tmesh);

// Vector Rotation using now face & new face's normal
Vec3d rotateVectorToNewNormal(
    Vec3d &vec,
    const Eigen::Vector3d &old_normal,
    const Eigen::Vector3d &new_normal);
// calculate basic informations between P & Q ->Geodesic distance, V_p, V_q
void geodesicbasecalcuation(const Eigen::Vector3d &p, const Eigen::Vector3d &q, Eigen::Vector3d &V_p, Eigen::Vector3d &V_q, double &geodesic_distance, const Triangle_mesh &tmesh,
                            const std::vector<TriangleFace> &mesh);
// Calculate Face normal vector
Eigen::Vector3d computeFaceNormal(
    const Vec3d &v1,
    const Vec3d &v2,
    const Vec3d &v3,
    bool clockwise = true);
// calculate angle between two vectors
double calculateAngleBetweenVectors(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &p, const Triangle_mesh &tmesh);

// find vector V_q has same angle with [V_p betwwen V_pq] at Q.
Eigen::Vector3d geodesicextend(const Eigen::Vector3d &p,
                               const Eigen::Vector3d &q,
                               const Eigen::Vector3d &V_q,
                               const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh, double angle);

// project P to face and find next point(intersect with face edge)
std::tuple<Vec3d, Vec3d> project_and_find_intersection(
    const Vec3d &current_point,
    const Vec3d &current_direction, double &distance_traveled,
    const Triangle_mesh &tmesh,
    const std::vector<TriangleFace> &mesh);

// using project_and_find_intersection(), find final_point that start from q, direction is start_direction_p.
Eigen::Vector3d geodesicAddVector(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &start_direction_p,
    double total_distance,
    const Eigen::Vector3d &q,
    const Triangle_mesh &tmesh,
    const std::vector<TriangleFace> &mesh);

// Calculate GeodesicDistance between two points
double computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh);

// Calculate Geodesic interpolation parameters
std::vector<double> calculateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points, bool chord_length);

// Geodesic subtraction of points in the geodesic domain
Eigen::Vector3d geodesicSubtract(
    const Eigen::Vector3d &p1,
    const Eigen::Vector3d &p2,
    const Triangle_mesh &tmesh);

// Calculate Geodesic tangent vectors
std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const Triangle_mesh &tmesh);

// Calculate Bézier spline's control points
std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const std::vector<Eigen::Vector3d> &tangent_vectors,
    const Triangle_mesh &tmesh);

// using control points, calculate Bezier Curve.
std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(
    const std::vector<Eigen::Vector3d> &control_points,
    const Triangle_mesh &tmesh,
    int steps);

void generate_Geodesic_Path(const std::vector<Eigen::Vector3d> &points, Triangle_mesh &tmesh);

// Convert curve_points to ROS_points to publish
void generate_Hermite_Spline_path(
    std::vector<Eigen::Vector3d> &selected_points, Triangle_mesh &tmesh);

void generate_B_Spline_Path(const std::vector<Eigen::Vector3d> &points, Triangle_mesh &tmesh);

void generate_Catmull_Rom_Path(const std::vector<Eigen::Vector3d> &points, Triangle_mesh &tmesh);

// read stl file
bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh);

// project clicked_points to mesh surface
void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

void keyboardCallback(const std_msgs::String::ConstPtr &msg);

// 파일에 Waypoints와 Control Points 저장하는 함수
void saveToTextFile(const std::string &filename);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    ros::Subscriber keyboard_sub = nh.subscribe("nrs_command", 10, keyboardCallback);
    waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10);
    controlpoints_pub = nh.advertise<std_msgs::Float64MultiArray>("control_points", 10);

    std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/fenda_wrap.stl";

    // 파일 이름 설정 (웨이포인트와 컨트롤 포인트가 하나의 파일에 저장됨)
    std::string filename = "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/waypoints_and_controlpoints.txt";

    std::ifstream input(mesh_file_path, std::ios::binary);
    read_stl_file(input, tmesh);

    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(tmesh);

    ros::Rate r(30);

    while (ros::ok())
    {
        ros::spinOnce();
        if (selected_points.size() > 1 && start_path_generating)
        {
            if (use_spline && selected_points.size() > 2)
            {

                waypoints_msg.waypoints.clear();
                control_points_msg.data.clear();
                u_values = {0.0};
                tangent_vectors.clear();
                bezier_control_points.clear();
                generate_Hermite_Spline_path(selected_points, tmesh);
            }
            else if (use_straight)
            {
                waypoints_msg.waypoints.clear();
                generate_Geodesic_Path(selected_points, tmesh);
            }

            waypoints_pub.publish(waypoints_msg); // 경로 생성 후 퍼블리시
            //controlpoints_pub.publish(control_points_msg);

            // 생성된 웨이포인트 및 컨트롤 포인트 파일로 저장
            //saveToTextFile(filename);

            start_path_generating = false; // 모션 플래닝이 끝난 후 플래그를 false로 설정
            use_spline = false;
            use_straight = false;
        }

        r.sleep();
    }

    delete tree;
    delete shortest_paths;
    return 0;
}

// double * Vec3d
Vec3d operator*(double scalar, const Vec3d &vec)
{
    return {scalar * vec.x, scalar * vec.y, scalar * vec.z};
}

// Vec3d to Eigen::Vector3d
Vec3d EigenToVec3d(const Eigen::Vector3d &eigen_vec)
{
    return {eigen_vec.x(), eigen_vec.y(), eigen_vec.z()};
}

// Eigen::Vector3d to Vec3d
Eigen::Vector3d Vec3dToEigen(const Vec3d &vec)
{
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

// Kernel::Point_3 to Vec3d
Vec3d CGALPointToVec3d(const Point_3 &p)
{
    return Vec3d(p.x(), p.y(), p.z());
}

// Vec3d to Kernel::Point_3
Point_3 Vec3dToCGALPoint(const Vec3d &v)
{
    return Point_3(v.x, v.y, v.z);
}

std::vector<geometry_msgs::Point> projectPointsOntoMesh(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<geometry_msgs::Point> projected_points, offset_points;

    for (const auto &point : points)
    {
        Point_3 query(point.x, point.y, point.z + 0.05);
        auto closest_point = tree->closest_point(query);

        geometry_msgs::Point off_point;
        off_point.x = query.x();
        off_point.y = query.y();
        off_point.z = query.z();
        offset_points.push_back(off_point);

        geometry_msgs::Point ros_point;
        ros_point.x = closest_point.x();
        ros_point.y = closest_point.y();
        ros_point.z = closest_point.z();

        projected_points.push_back(ros_point);
    }
    // offset_point와 projected_point를 시각화합니다.
    // visualizePath(offset_points, "offset_B_spline_path", 1, 0.0, 0.0, 1.0, 1.0);
    // visualizePath(projected_points, "projected_B_spline_path", 1, 1.0, 0.0, 0.0, 1.0);

    return projected_points;
}

/*----------------------------------------------------------------------*/

// tranform triangle to basic calculation
std::vector<TriangleFace> convertMeshToTriangleFaces(const Triangle_mesh &tmesh)
{
    std::vector<TriangleFace> triangle_faces;

    for (auto face : tmesh.faces())
    {
        TriangleFace triangle;

        int i = 0;
        for (auto vertex : vertices_around_face(tmesh.halfedge(face), tmesh))
        {
            Kernel::Point_3 p = tmesh.point(vertex);
            triangle.vertices[i] = EigenToVec3d(Eigen::Vector3d(p.x(), p.y(), p.z()));
            i++;
        }

        triangle_faces.push_back(triangle);
    }

    return triangle_faces;
}

// Find closest [face and barycentric coordinate on Mesh] with points
bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &tmesh)
{
    if (tmesh.faces().empty())
    {
        std::cerr << "Mesh is empty, cannot build AABB tree." << std::endl;
        return false;
    }

    Tree tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree.build();

    auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, tmesh);
    face = result.first;
    location = result.second;

    if (face == Triangle_mesh::null_face())
    {
        std::cerr << "Failed to locate face for point: " << point << std::endl;
        return false;
    }

    return true;
}

// points to waypoints to publish
nrs_vision_rviz::Waypoints convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh tmesh)
{
    nrs_vision_rviz::Waypoints waypoints;

    for (const auto &point : points)
    {
        // Point_3 cgal_point(point.x, point.y, point.z);
        // face_descriptor face;
        // Surface_mesh_shortest_path::Barycentric_coordinates location;

        // if (!locate_face_and_point(cgal_point, face, location, tmesh))
        // {
        //     ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", point.x, point.y, point.z);
        //     continue;
        // }

        // Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, tmesh);

        nrs_vision_rviz::Waypoint waypoint_msg;

        waypoint_msg.pose.position.x = point.x;
        waypoint_msg.pose.position.y = point.y;
        waypoint_msg.pose.position.z = point.z;

        waypoint_msg.pose.orientation.w = 0;
        waypoint_msg.pose.orientation.x = 0;
        waypoint_msg.pose.orientation.y = 0;
        waypoint_msg.pose.orientation.z = 0;

        // waypoint_msg.normal.x = -normal.x();
        // waypoint_msg.normal.y = -normal.y();
        // waypoint_msg.normal.z = -normal.z();

        waypoints.waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}

// Vector Rotation using now face & new face's normal
Vec3d rotateVectorToNewNormal(
    Vec3d &vec,
    const Eigen::Vector3d &old_normal,
    const Eigen::Vector3d &new_normal)
{
    Eigen::Vector3d v = Vec3dToEigen(vec);
    Eigen::Vector3d rotation_axis = old_normal.cross(new_normal);
    double angle = acos(old_normal.dot(new_normal) / (old_normal.norm() * new_normal.norm()));

    if (rotation_axis.norm() < 1e-6 || std::isnan(angle))
    {

        Vec3d v2 = EigenToVec3d(v);
        return v2;
    }

    Eigen::AngleAxisd rotation(angle, rotation_axis.normalized());
    Eigen::Vector3d rotated_vec = rotation * v;
    rotated_vec = rotated_vec.normalized();
    Vec3d rotated_vec2 = EigenToVec3d(rotated_vec);

    return rotated_vec2;
}

// calculate basic informations between P & Q ->Geodesic distance, V_p, V_q
void geodesicbasecalcuation(const Eigen::Vector3d &p, const Eigen::Vector3d &q, Eigen::Vector3d &V_p, Eigen::Vector3d &V_q, double &geodesic_distance, const Triangle_mesh &tmesh,
                            const std::vector<TriangleFace> &mesh)
{
    Kernel::Point_3 point1(p.x(), p.y(), p.z());
    Kernel::Point_3 point2(q.x(), q.y(), q.z());

    face_descriptor face1, face2;
    Surface_mesh_shortest_path::Barycentric_coordinates location1, location2;

    if (!locate_face_and_point(point1, face1, location1, tmesh))
    {
        throw std::runtime_error("Failed to locate point1 on mesh.");
    }
    if (!locate_face_and_point(point2, face2, location2, tmesh))
    {
        throw std::runtime_error("Failed to locate point2 on mesh.");
    }

    // if (face1 != face2)
    // {

    Surface_mesh_shortest_path shortest_paths(tmesh);
    shortest_paths.add_source_point(face2, location2);

    std::vector<Surface_mesh_shortest_path::Point_3> path_points;
    shortest_paths.shortest_path_points_to_source_points(face1, location1, std::back_inserter(path_points));

    std::pair<double, Surface_mesh_shortest_path::Source_point_iterator> result = shortest_paths.shortest_distance_to_source_points(face1, location1);
    geodesic_distance = result.first;

    if (path_points.size() < 2)
    {
        throw std::runtime_error("Geodesic path does not contain enough points.");
    }

    auto halfedge1 = tmesh.halfedge(face1);
    Kernel::Point_3 vp0 = tmesh.point(tmesh.source(halfedge1));
    Kernel::Point_3 vp1 = tmesh.point(tmesh.target(halfedge1));
    Kernel::Point_3 vp2 = tmesh.point(tmesh.target(tmesh.next(halfedge1)));

    auto halfedge2 = tmesh.halfedge(face2);
    Kernel::Point_3 vq0 = tmesh.point(tmesh.source(halfedge2));
    Kernel::Point_3 vq1 = tmesh.point(tmesh.target(halfedge2));
    Kernel::Point_3 vq2 = tmesh.point(tmesh.target(tmesh.next(halfedge2)));

    Eigen::Vector3d point_p1(path_points[1].x(), path_points[1].y(), path_points[1].z());

    double epsilon = 1e-6;

    Eigen::Vector3d last_point(path_points[path_points.size() - 1].x(), path_points[path_points.size() - 1].y(), path_points[path_points.size() - 1].z());
    Eigen::Vector3d second_last_point(path_points[path_points.size() - 2].x(), path_points[path_points.size() - 2].y(), path_points[path_points.size() - 2].z());

    if ((last_point - second_last_point).norm() < epsilon)
    {

        Eigen::Vector3d point_q0(path_points[path_points.size() - 3].x(), path_points[path_points.size() - 3].y(), path_points[path_points.size() - 3].z());

        V_q = q - point_q0;
    }
    else
    {

        Eigen::Vector3d point_q0(path_points[path_points.size() - 2].x(), path_points[path_points.size() - 2].y(), path_points[path_points.size() - 2].z());

        V_q = q - point_q0;
    }

    V_p = point_p1 - p;

    V_p.normalize();
    V_q.normalize();
    // }
    // else
    // {

    //     V_p = q - p;
    //     V_q = q - p;
    //     V_p.normalize();
    //     V_q.normalize();
    //     geodesic_distance = V_p.norm();
    // }
}

// Calculate Face normal vector
Eigen::Vector3d computeFaceNormal(
    const Vec3d &v1,
    const Vec3d &v2,
    const Vec3d &v3,
    bool clockwise)
{
    Eigen::Vector3d edge1 = Vec3dToEigen(v2) - Vec3dToEigen(v1);
    Eigen::Vector3d edge2 = Vec3dToEigen(v3) - Vec3dToEigen(v1);

    Eigen::Vector3d normal;
    if (clockwise)
    {
        normal = edge1.cross(edge2);
    }
    else
    {
        normal = edge2.cross(edge1);
    }

    return normal.normalized();
}

// calculate angle between two vectors
double calculateAngleBetweenVectors(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &p, const Triangle_mesh &tmesh)
{

    face_descriptor face_desc;
    Surface_mesh_shortest_path::Barycentric_coordinates bary_coords;

    if (!locate_face_and_point(Vec3dToCGALPoint(Vec3d(p.x(), p.y(), p.z())), face_desc, bary_coords, tmesh))
    {
        std::cerr << "Error: Failed to locate the face for the point." << std::endl;
        return 0.0;
    }

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(face_desc), tmesh);
    auto v_it = vertices.begin();

    Vec3d v1 = CGALPointToVec3d(tmesh.point(*v_it++));
    Vec3d v2 = CGALPointToVec3d(tmesh.point(*v_it++));
    Vec3d v3 = CGALPointToVec3d(tmesh.point(*v_it));

    Eigen::Vector3d face_normal = computeFaceNormal(v1, v2, v3);

    double dot_product = vec1.dot(vec2);

    double magnitude_vec1 = vec1.norm();
    double magnitude_vec2 = vec2.norm();

    if (magnitude_vec1 == 0 || magnitude_vec2 == 0)
    {
        std::cerr << "Error: One of the vectors has zero length, cannot compute angle." << std::endl;
        return 0.0;
    }

    double cos_theta = dot_product / (magnitude_vec1 * magnitude_vec2);

    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

    double angle_rad = acos(cos_theta);

    Eigen::Vector3d cross_product = vec1.cross(vec2);

    double direction = cross_product.dot(face_normal);

    if (direction < 0)
    {
        angle_rad = -angle_rad;
    }

    double angle_deg = angle_rad * (180.0 / M_PI);

    return angle_rad;
}

// find vector V_q has same angle with [V_p betwwen V_pq] at Q.
Eigen::Vector3d geodesicextend(const Eigen::Vector3d &p,
                               const Eigen::Vector3d &q,
                               const Eigen::Vector3d &V_q,
                               const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh, double angle)
{
    Kernel::Point_3 point1(p.x(), p.y(), p.z());
    Kernel::Point_3 point2(q.x(), q.y(), q.z());

    face_descriptor face1, face2;
    Surface_mesh_shortest_path::Barycentric_coordinates location1, location2;

    if (!locate_face_and_point(point1, face1, location1, tmesh))
    {
        throw std::runtime_error("Failed to locate point1 on mesh.");
    }
    if (!locate_face_and_point(point2, face2, location2, tmesh))
    {
        throw std::runtime_error("Failed to locate point2 on mesh.");
    }

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(face2), tmesh);
    auto v_it = vertices.begin();

    Point_3 v1 = tmesh.point(*v_it++);
    Point_3 v2 = tmesh.point(*v_it++);
    Point_3 v3 = tmesh.point(*v_it);

    Vec3d v1_vec = CGALPointToVec3d(v1);
    Vec3d v2_vec = CGALPointToVec3d(v2);
    Vec3d v3_vec = CGALPointToVec3d(v3);

    Eigen::Vector3d normal = computeFaceNormal(v1_vec, v2_vec, v3_vec);

    if (normal.norm() == 0)
    {
        throw std::runtime_error("Invalid normal vector, cannot perform rotation.");
    }

    Eigen::Vector3d rotated_vector;

    Eigen::AngleAxisd rotation(-angle, normal.normalized());
    rotated_vector = rotation * V_q;

    return rotated_vector;
}

// project P to face and find next point(intersect with face edge)
std::tuple<Vec3d, Vec3d> project_and_find_intersection(
    const Vec3d &current_point,
    const Vec3d &current_direction, double &distance_traveled,
    const Triangle_mesh &tmesh,
    const std::vector<TriangleFace> &mesh)
{
    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
    Vec3d updated_point = current_point + epsilon_vector;

    locate_face_and_point(Vec3dToCGALPoint(updated_point), current_face_descriptor, barycentric_coords, tmesh);

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
    auto v_it = vertices.begin();

    Point_3 v1 = tmesh.point(*v_it++);
    Point_3 v2 = tmesh.point(*v_it++);
    Point_3 v3 = tmesh.point(*v_it);

    Vec3d v1_vec = CGALPointToVec3d(v1);
    Vec3d v2_vec = CGALPointToVec3d(v2);
    Vec3d v3_vec = CGALPointToVec3d(v3);

    Vec3d projected_point = current_point;
    Vec3d projected_direction = current_direction.normalize();

    Eigen::Matrix2d T;
    T << (v2_vec - v1_vec).x, (v3_vec - v1_vec).x,
        (v2_vec - v1_vec).y, (v3_vec - v1_vec).y;

    Eigen::Vector2d bary_p0 = T.inverse() * Eigen::Vector2d(projected_point.x - v1_vec.x, projected_point.y - v1_vec.y);
    Eigen::Vector2d bary_direction = T.inverse() * Eigen::Vector2d(projected_direction.x, projected_direction.y);

    double t_intersect = std::numeric_limits<double>::max();
    Eigen::Vector2d bary_intersection;

    if (bary_direction.x() != 0)
    {
        double t1 = -bary_p0.x() / bary_direction.x();
        Eigen::Vector2d bary1 = bary_p0 + t1 * bary_direction;
        if (t1 >= 0 && t1 < t_intersect && bary1.y() >= 0 && bary1.y() <= 1)
        {
            t_intersect = t1;
            bary_intersection = bary1;
        }
    }

    if (bary_direction.y() != 0)
    {
        double t2 = -bary_p0.y() / bary_direction.y();
        Eigen::Vector2d bary2 = bary_p0 + t2 * bary_direction;
        if (t2 >= 0 && t2 < t_intersect && bary2.x() >= 0 && bary2.x() <= 1)
        {
            t_intersect = t2;
            bary_intersection = bary2;
        }
    }

    double denom = bary_direction.x() + bary_direction.y();
    if (denom != 0)
    {
        double t3 = (1 - bary_p0.x() - bary_p0.y()) / denom;
        Eigen::Vector2d bary3 = bary_p0 + t3 * bary_direction;
        if (t3 >= 0 && t3 < t_intersect && bary3.x() >= 0 && bary3.y() >= 0)
        {
            t_intersect = t3;
            bary_intersection = bary3;
        }
    }

    Vec3d final_point = v1_vec + bary_intersection.x() * (v2_vec - v1_vec) + bary_intersection.y() * (v3_vec - v1_vec);

    Vec3d new_direction = (final_point - current_point).normalize();

    const double epsilon = 1e-6;

    if ((std::abs(final_point.x - current_point.x) < epsilon) &&
        (std::abs(final_point.y - current_point.y) < epsilon) &&
        (std::abs(final_point.z - current_point.z) < epsilon))
    {

        Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
        Vec3d offset_point = current_point + epsilon_vector;

        auto [updated_point, updated_direction] = project_and_find_intersection(offset_point, current_direction, distance_traveled, tmesh, mesh);

        return std::make_tuple(updated_point, updated_direction);
    }
    else
    {

        return std::make_tuple(final_point, new_direction);
    }
}

// using project_and_find_intersection(), find final_point that start from q, direction is start_direction_p.
Eigen::Vector3d geodesicAddVector(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &start_direction_p,
    double total_distance,
    const Eigen::Vector3d &q,
    const Triangle_mesh &tmesh,
    const std::vector<TriangleFace> &mesh)
{

    Eigen::Vector3d V_p;
    Eigen::Vector3d V_q;
    double geodesic_distance;
    double distance_traveled = 0.0;
    Eigen::Vector3d start_direction;
    double percentage_traveled;
    Vec3d final_point;

    if (p == q)
    {

        start_direction = start_direction_p;
    }
    else
    {
        geodesicbasecalcuation(p, q, V_p, V_q, geodesic_distance, tmesh, mesh);

        double angle1 = calculateAngleBetweenVectors(start_direction_p, V_p, p, tmesh);

        start_direction = geodesicextend(p, q, V_q, tmesh, mesh, angle1);
    }

    if (total_distance == 0)
    {
        return q;
    }

    Vec3d current_point = EigenToVec3d(q);
    Vec3d current_direction = EigenToVec3d(start_direction).normalize();

    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    if (!locate_face_and_point(Vec3dToCGALPoint(current_point), current_face_descriptor, barycentric_coords, tmesh))
    {
        std::cerr << "Failed to locate point on mesh." << std::endl;
        return Vec3dToEigen(current_point);
    }

    while (true)
    {

        auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
        auto v_it = vertices.begin();

        Point_3 v1 = tmesh.point(*v_it++);
        Point_3 v2 = tmesh.point(*v_it++);
        Point_3 v3 = tmesh.point(*v_it);

        Vec3d v1_vec = CGALPointToVec3d(v1);
        Vec3d v2_vec = CGALPointToVec3d(v2);
        Vec3d v3_vec = CGALPointToVec3d(v3);

        Eigen::Vector3d current_normal = computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        auto [new_point, new_direction] = project_and_find_intersection(current_point, current_direction, distance_traveled, tmesh, mesh);

        face_descriptor new_face_descriptor;

        locate_face_and_point(Vec3dToCGALPoint(new_point), new_face_descriptor, barycentric_coords, tmesh);

        if (new_face_descriptor == current_face_descriptor)
        {

            Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
            Vec3d updated_point = new_point + epsilon_vector;

            if (!locate_face_and_point(Vec3dToCGALPoint(updated_point), new_face_descriptor, barycentric_coords, tmesh))
            {
                std::cerr << "Failed to locate new point on mesh." << std::endl;
                return Vec3dToEigen(updated_point);
            }
        }

        vertices = CGAL::vertices_around_face(tmesh.halfedge(new_face_descriptor), tmesh);
        v_it = vertices.begin();

        v1 = tmesh.point(*v_it++);
        v2 = tmesh.point(*v_it++);
        v3 = tmesh.point(*v_it);

        v1_vec = CGALPointToVec3d(v1);
        v2_vec = CGALPointToVec3d(v2);
        v3_vec = CGALPointToVec3d(v3);

        Eigen::Vector3d new_normal = computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        new_direction = rotateVectorToNewNormal(new_direction, current_normal, new_normal);

        double new_distance_traveled = sqrt((new_point.x - current_point.x) * (new_point.x - current_point.x) + (new_point.y - current_point.y) * (new_point.y - current_point.y) + (new_point.z - current_point.z) * (new_point.z - current_point.z));

        if (distance_traveled + new_distance_traveled >= abs(total_distance))
        {
            double remaining_distance = abs(total_distance) - distance_traveled;
            final_point = current_point + current_direction.normalize() * remaining_distance;

            distance_traveled += remaining_distance;

            break;
        }

        current_face_descriptor = new_face_descriptor;
        current_point = new_point;
        current_direction = new_direction;
        distance_traveled += new_distance_traveled;
    }

    return Vec3dToEigen(final_point);
}

// Calculate GeodesicDistance between two points
double computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh)
{

    Kernel::Point_3 point0(p0.x(), p0.y(), p0.z());
    Kernel::Point_3 point1(p1.x(), p1.y(), p1.z());

    face_descriptor face0, face1;
    Surface_mesh_shortest_path::Barycentric_coordinates location0, location1;

    if (!locate_face_and_point(point0, face0, location0, mesh))
    {
        throw std::runtime_error("Failed to locate point0 on mesh.");
    }

    if (!locate_face_and_point(point1, face1, location1, mesh))
    {
        throw std::runtime_error("Failed to locate point1 on mesh.");
    }

    Surface_mesh_shortest_path shortest_paths(mesh);
    shortest_paths.add_source_point(face1, location1);

    std::pair<double, Surface_mesh_shortest_path::Source_point_iterator> result = shortest_paths.shortest_distance_to_source_points(face0, location0);

    return abs(result.first);
}

// Calculate Geodesic interpolation parameters
std::vector<double> calculateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points, bool chord_length)
{
    for (int i = 1; i < selected_points.size(); i++)
    {
        double geodesic_distance = computeGeodesicDistance(selected_points[i], selected_points[i + 1], tmesh);

        if (!chord_length)
        {
            geodesic_distance = std::sqrt(geodesic_distance);
        }

        double ui = u_values.back() + geodesic_distance;
        u_values.push_back(ui);
    }
    return u_values;
}

// Geodesic subtraction of points in the geodesic domain
Eigen::Vector3d geodesicSubtract(
    const Eigen::Vector3d &p1,
    const Eigen::Vector3d &p2,
    const Triangle_mesh &tmesh)
{
    Eigen::Vector3d V_p, V_q;
    double geodesic_distance;
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    geodesicbasecalcuation(p1, p2, V_p, V_q, geodesic_distance, tmesh, mesh);

    return V_p * geodesic_distance;
}

// Calculate Geodesic tangent vectors
std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const Triangle_mesh &tmesh)
{
    float c = 0.6;

    std::vector<Eigen::Vector3d> tangent_vectors;

    if (selected_points.empty() || u_values.empty())
    {
        throw std::runtime_error("selected_points or u_values are empty!");
    }

    Eigen::Vector3d p_first = selected_points.front();
    Eigen::Vector3d p_last = selected_points.back();

    if (p_first == p_last)
    {

        Eigen::Vector3d p_prev = selected_points[1];
        Eigen::Vector3d p_now = selected_points[0];
        Eigen::Vector3d p_next = selected_points[selected_points.size() - 2];

        Eigen::Vector3d tangent_vector = (1 - c) * (geodesicSubtract(p_prev, p_next, tmesh)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));

        tangent_vectors.push_back(tangent_vector);
    }
    else
    {

        tangent_vectors.push_back(Eigen::Vector3d::Zero());
    }

    for (size_t i = 1; i < selected_points.size() - 1; ++i)
    {
        Eigen::Vector3d p_prev = selected_points[i - 1];
        Eigen::Vector3d p_now = selected_points[i];
        Eigen::Vector3d p_next = selected_points[i + 1];

        Eigen::Vector3d tangent_vector = (1 - c) * (geodesicSubtract(p_prev, p_next, tmesh)) / (u_values[i + 1] - u_values[i - 1]);
        tangent_vectors.push_back(tangent_vector);
    }

    if (p_first == p_last)
    {

        Eigen::Vector3d p_prev = selected_points[1];
        Eigen::Vector3d p_now = selected_points[0];
        Eigen::Vector3d p_next = selected_points[selected_points.size() - 2];
        Eigen::Vector3d tangent_vector = (1 - c) * (geodesicSubtract(p_prev, p_next, tmesh)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));
        tangent_vectors.push_back(tangent_vector);
    }
    else
    {

        tangent_vectors.push_back(Eigen::Vector3d::Zero());
    }

    return tangent_vectors;
}

// Calculate Bézier spline's control points
std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const std::vector<Eigen::Vector3d> &tangent_vectors,
    const Triangle_mesh &tmesh)
{
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    std::vector<std::vector<Eigen::Vector3d>> bezier_control_points;
    int marker_id = 0;

    Eigen::Vector3d p_first = selected_points.front();
    Eigen::Vector3d p_last = selected_points.back();

    Eigen::Vector3d p_prev = selected_points[selected_points.size() - 2];
    Eigen::Vector3d p_now = selected_points[0];
    Eigen::Vector3d p_next = selected_points[1];

    Eigen::Vector3d tangent_prev = tangent_vectors[tangent_vectors.size() - 2];
    Eigen::Vector3d tangent_now = tangent_vectors[0];
    Eigen::Vector3d tangent_next = tangent_vectors[1];
    double distance = (u_values[1] - u_values[0]) / 3.0;

    if (p_first == p_last)
    {

        Eigen::Vector3d b0 = p_now;
        Eigen::Vector3d b1 = geodesicAddVector(p_prev, tangent_now, distance * tangent_now.norm(), p_now, tmesh, mesh);
        Eigen::Vector3d b2 = geodesicAddVector(p_now, -tangent_next, distance * tangent_next.norm(), p_next, tmesh, mesh);
        Eigen::Vector3d b3 = p_next;
        bezier_control_points.push_back({b0, b1, b2, b3});
    }
    else
    {

        double distance = (u_values[1] - u_values[0]) / 3.0;
        Eigen::Vector3d b2 = geodesicAddVector(p_now, -tangent_next, distance * tangent_next.norm(), p_next, tmesh, mesh);
        bezier_control_points.push_back({p_now, p_now, b2, p_next});
    }

    for (size_t i = 1; i < selected_points.size() - 2; ++i)
    {
        Eigen::Vector3d p_prev = selected_points[i - 1];
        Eigen::Vector3d p_now = selected_points[i];
        Eigen::Vector3d p_next = selected_points[i + 1];

        Eigen::Vector3d tangent_prev = tangent_vectors[i - 1];
        Eigen::Vector3d tangent_now = tangent_vectors[i];
        Eigen::Vector3d tangent_next = tangent_vectors[i + 1];

        double distance = (u_values[i + 1] - u_values[i]) / 3.0;
        Eigen::Vector3d b0 = p_now;
        Eigen::Vector3d b1 = geodesicAddVector(p_prev, tangent_now, distance * tangent_now.norm(), p_now, tmesh, mesh);
        Eigen::Vector3d b2 = geodesicAddVector(p_now, -tangent_next, distance * tangent_next.norm(), p_next, tmesh, mesh);
        Eigen::Vector3d b3 = p_next;
        bezier_control_points.push_back({b0, b1, b2, b3});
    }

    if (p_first == p_last)
    {

        Eigen::Vector3d b0 = p_prev;
        double distance = (u_values[0] - u_values[u_values.size() - 2]) / 3.0;
        Eigen::Vector3d b1 = geodesicAddVector(p_next, tangent_prev, distance * tangent_prev.norm(), p_prev, tmesh, mesh);
        Eigen::Vector3d b2 = geodesicAddVector(p_prev, -tangent_now, distance * tangent_now.norm(), p_now, tmesh, mesh);
        Eigen::Vector3d b3 = p_now;
        bezier_control_points.push_back({b0, b1, b2, b3});
    }
    else
    {

        double distance = (u_values[u_values.size() - 1] - u_values[u_values.size() - 2]) / 3.0;
        Eigen::Vector3d b1 = geodesicAddVector(selected_points[selected_points.size() - 3], tangent_vectors[tangent_vectors.size() - 2], distance * tangent_prev.norm(), selected_points[selected_points.size() - 2], tmesh, mesh);

        bezier_control_points.push_back({selected_points[selected_points.size() - 2], b1, selected_points[selected_points.size() - 1], selected_points[selected_points.size() - 1]});
    }
    std::cout << "Computing Bezier control points complete " << std::endl;

    // Bezier 제어점을 메시지로 변환
    for (const auto &control_point_set : bezier_control_points)
    {
        for (const auto &point : control_point_set)
        {
            control_points_msg.data.push_back(point.x());
            control_points_msg.data.push_back(point.y());
            control_points_msg.data.push_back(point.z());
        }
    }

    return bezier_control_points;
}

// using control points, calculate Bezier Curve.
std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(
    const std::vector<Eigen::Vector3d> &control_points,
    const Triangle_mesh &tmesh,
    int steps)
{
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    std::vector<Eigen::Vector3d> curve_points;
    Eigen::Vector3d V_b0, V_q0;
    double geodesic_distance;
    curve_points.reserve(steps + 1);
    Eigen::Vector3d b0 = control_points[0];
    Eigen::Vector3d b1 = control_points[1];
    Eigen::Vector3d b2 = control_points[2];
    Eigen::Vector3d b3 = control_points[3];

    Eigen::Vector3d v01 = geodesicSubtract(b0, b1, tmesh);
    Eigen::Vector3d v02 = geodesicSubtract(b0, b2, tmesh);
    Eigen::Vector3d v03 = geodesicSubtract(b0, b3, tmesh);

    for (int i = 0; i < steps; ++i)
    {

        double t = static_cast<double>(i) / steps;

        Eigen::Vector3d q0 = geodesicAddVector(b0, v01.normalized(), 3 * (1 - t) * (1 - t) * t * v01.norm(), b0, tmesh, mesh);

        Eigen::Vector3d q1 = geodesicAddVector(b0, v02.normalized(), 3 * (1 - t) * t * t * v02.norm(), q0, tmesh, mesh);

        Eigen::Vector3d q2 = geodesicAddVector(b0, v03.normalized(), t * t * t * v03.norm(), q1, tmesh, mesh);

        curve_points.push_back(q2);

    }

    return curve_points;
}

void generate_Geodesic_Path(const std::vector<Eigen::Vector3d> &points, Triangle_mesh &tmesh)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Point_3> complete_path;
    std::vector<Point_3> path_segment;

    for (size_t i = 1; i < points.size(); ++i)
    {
        Point_3 start(points[i - 1].x(), points[i - 1].y(), points[i - 1].z());
        Point_3 end(points[i].x(), points[i].y(), points[i].z());

        face_descriptor start_face, end_face;
        Surface_mesh_shortest_path::Barycentric_coordinates start_location, end_location;

        locate_face_and_point(start, start_face, start_location, tmesh);
        locate_face_and_point(end, end_face, end_location, tmesh);
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
    waypoints_msg = convertToWaypoints(path_points, tmesh);

    // 프로그램 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();

    // 시작과 종료 시간의 차이 계산 (밀리초 단위)
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    // 소요 시간 출력
    std::cout << "straight path generation time: " << duration << " s" << std::endl;
}

// Convert curve_points to ROS_points to publish
void generate_Hermite_Spline_path(
    std::vector<Eigen::Vector3d> &selected_points, Triangle_mesh &tmesh)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector3d> hermite_spline;
    if (selected_points.size() > 2)
    {
        u_values = calculateInterpolationParameters(selected_points, false);
        tangent_vectors = calculateGeodesicTangentVectors(selected_points, u_values, tmesh);
        bezier_control_points = computeBezierControlPoints(selected_points, u_values, tangent_vectors, tmesh);

        int i = 0;
        for (const auto &control_points : bezier_control_points)
        {
            std::cout << "generating Spline bewteen point[" << i << "] and point[" << i + 1 << "]" << std::endl;
            waypoints_distance += computeGeodesicDistance(selected_points[i], selected_points[i + 1], tmesh);
            steps = waypoints_distance * 50;
            std::vector<Eigen::Vector3d> curve_points = computeGeodesicBezierCurvePoints(control_points, tmesh, steps);
            hermite_spline.insert(hermite_spline.end(), curve_points.begin(), curve_points.end());
            i += 1;
        }
    }

    std::vector<geometry_msgs::Point> path_points;
    for (const auto &point : hermite_spline)
    {
        geometry_msgs::Point ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_point.z = point.z();
        path_points.push_back(ros_point);
    }

    ROS_INFO("Generated Hermite_Spline path with %zu points", path_points.size());
    waypoints_msg = convertToWaypoints(path_points, tmesh);

    // 프로그램 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();

    // 시작과 종료 시간의 차이 계산 (밀리초 단위)
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    // 소요 시간 출력
    std::cout << "spline path generation time: " << duration << " s" << std::endl;
}

void generate_B_Spline_Path(const std::vector<Eigen::Vector3d> &points, Triangle_mesh &tmesh)
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

        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = points[i].x();
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = points[i].y();
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = points[i].z();

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = points[i + 1].x();
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = points[i + 1].y();
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = points[i + 1].z();

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
    waypoints_msg = convertToWaypoints(smooth_path, tmesh);
}

void generate_Catmull_Rom_Path(const std::vector<Eigen::Vector3d> &points, Triangle_mesh &tmesh)
{
    // Eigen::Vector3d를 geometry_msgs::Point로 변환
    std::vector<geometry_msgs::Point> geom_points;
    for (const auto &eigen_point : points)
    {
        geometry_msgs::Point geom_point;
        geom_point.x = eigen_point.x();
        geom_point.y = eigen_point.y();
        geom_point.z = eigen_point.z();
        geom_points.push_back(geom_point);
    }

    std::vector<geometry_msgs::Point> smooth_path;
    if (geom_points.size() < 4)
        return; // Catmull-Rom spline을 위해 최소 4개의 점이 필요

    for (size_t i = 1; i < geom_points.size() - 2; ++i)
    {
        geometry_msgs::Point p0 = geom_points[i - 1];
        geometry_msgs::Point p1 = geom_points[i];
        geometry_msgs::Point p2 = geom_points[i + 1];
        geometry_msgs::Point p3 = geom_points[i + 2];

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

    // mesh에 투영된 점을 계산
    std::vector<geometry_msgs::Point> projected_path = projectPointsOntoMesh(smooth_path);

    // Waypoints로 변환하여 메시지에 저장
    waypoints_msg = convertToWaypoints(projected_path, tmesh);
}

// read stl file
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
        if (mesh.add_face(index_to_vertex[t[0]], index_to_vertex[t[1]], index_to_vertex[t[2]]) == Triangle_mesh::null_face())
        {
            ROS_ERROR("Failed to add face.");
            return false;
        }
    }
    ROS_INFO("Successfully read STL file.");
    return true;
}

// project clicked_points to mesh surface
void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    std::cout << "-----------------------------------new waypoints comming---------------------------------" << std::endl;

    Kernel::Point_3 clicked_point(msg->point.x, msg->point.y, msg->point.z);

    face_descriptor face;
    Surface_mesh_shortest_path::Barycentric_coordinates location;

    if (!locate_face_and_point(clicked_point, face, location, tmesh))
    {
        std::cerr << "Failed to locate the clicked point on the mesh." << std::endl;
        return;
    }

    Kernel::Point_3 v1, v2, v3;
    int i = 0;

    for (auto v : vertices_around_face(tmesh.halfedge(face), tmesh))
    {
        if (i == 0)
            v1 = tmesh.point(v);
        else if (i == 1)
            v2 = tmesh.point(v);
        else if (i == 2)
            v3 = tmesh.point(v);

        ++i;
    }

    Kernel::Point_3 projected_point = CGAL::barycenter(v1, location[0], v2, location[1], v3, location[2]);

    Eigen::Vector3d projected_point_eigen(projected_point.x(), projected_point.y(), projected_point.z());

    clicked_points.push_back(msg->point);
    selected_points.push_back(projected_point_eigen);
}

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "generate_Geodesic_Path")
    {
        start_path_generating = true;
        use_straight = true;
        ROS_INFO("Received start command, initiating generate_Geodesic_Path planning.");
    }
    if (msg->data == "reset")
    {
        clicked_points.clear();
        selected_points.clear();

        ROS_INFO("waypoints cleared");
    }

    if (msg->data == "generate_Hermite_Spline_path")
    {
        start_path_generating = true;
        use_spline = true;
        ROS_INFO("Received start command, initiating generate_Hermite_Spline_path planning.");
    }
}

// 파일에 Waypoints와 Control Points 저장하는 함수
void saveToTextFile(const std::string &filename)
{
    // 파일 열기
    std::ofstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("Unable to open file: %s", filename.c_str());
        return;
    }

    // Waypoints 저장
    for (const auto &waypoint : waypoints_msg.waypoints)
    {
        // Position (x, y, z) 저장
        file << "waypoint: " << waypoint.pose.position.x << " "
             << waypoint.pose.position.y << " "
             << waypoint.pose.position.z << "\n";
    }

    // Control Points 저장
    for (size_t i = 0; i < control_points_msg.data.size(); i += 3)
    {
        file << "controlpoint: " << control_points_msg.data[i] << " "
             << control_points_msg.data[i + 1] << " "
             << control_points_msg.data[i + 2] << "\n";
    }

    ROS_INFO("Waypoints and control points saved to %s", filename.c_str());
    file.close();
}