#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// CGAL 관련 헤더
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL_reader.h>
#include <boost/lexical_cast.hpp>

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

std::vector<geometry_msgs::Point> clicked_points;
bool new_waypoints = false;
Triangle_mesh mesh;
Tree *tree;
Surface_mesh_shortest_path *shortest_paths;
nrs_vision_rviz::Waypoints waypoints_msg;
ros::Publisher waypoints_pub;
std::vector<Eigen::Vector3d> selected_points; // 선택한 점들
std::vector<double> u_values = {0.0};         // u_0 = 0으로 초기화
std::vector<Eigen::Vector3d> tangent_vectors; // 선택한 점들의 tangent vectors
ros::Publisher marker_pub2;

// 기본 3D 벡터 연산
struct Vec3d
{
    double x, y, z;
    // 기본 생성자
    Vec3d() : x(0), y(0), z(0) {}

    // 3개의 double 인자를 받는 생성자 추가
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

// 연산자 오버로딩을 통해 double * Vec3d 지원
Vec3d operator*(double scalar, const Vec3d &vec)
{
    return {scalar * vec.x, scalar * vec.y, scalar * vec.z};
}

// Vec3d와 Eigen::Vector3d 간의 변환 함수
Vec3d EigenToVec3d(const Eigen::Vector3d &eigen_vec)
{
    return {eigen_vec.x(), eigen_vec.y(), eigen_vec.z()};
}

Eigen::Vector3d Vec3dToEigen(const Vec3d &vec)
{
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

// 삼각형 면 정의
struct TriangleFace
{
    Vec3d vertices[3];
    Vec3d normal;
    const TriangleFace *neighbors[3];
};

std::vector<TriangleFace> convertMeshToTriangleFaces(const Triangle_mesh &mesh)
{
    std::vector<TriangleFace> triangle_faces;

    for (auto face : mesh.faces())
    {
        TriangleFace triangle;

        int i = 0;
        for (auto vertex : vertices_around_face(mesh.halfedge(face), mesh))
        {
            Kernel::Point_3 p = mesh.point(vertex);
            triangle.vertices[i] = EigenToVec3d(Eigen::Vector3d(p.x(), p.y(), p.z()));
            i++;
        }

        triangle_faces.push_back(triangle);
    }

    return triangle_faces;
}

// 화살표를 시각화하는 함수
void visualizeDirectionArrow(const Eigen::Vector3d &start_point, const Eigen::Vector3d &direction, visualization_msgs::MarkerArray &marker_array, int marker_id)
{
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "base_link";
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.ns = "geodesic_direction";
    arrow_marker.id = marker_id;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start;
    start.x = start_point.x();
    start.y = start_point.y();
    start.z = start_point.z();
    arrow_marker.points.push_back(start);

    geometry_msgs::Point end;
    end.x = start_point.x() + direction.x();
    end.y = start_point.y() + direction.y();
    end.z = start_point.z() + direction.z();
    arrow_marker.points.push_back(end);

    arrow_marker.scale.x = 0.001;
    arrow_marker.scale.y = 0.001;
    arrow_marker.scale.z = 0.001;

    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.0;

    marker_array.markers.push_back(arrow_marker);
}

// 점을 시각화하는 함수
void visualizePoint(const Eigen::Vector3d &point, visualization_msgs::MarkerArray &marker_array, int marker_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path_points";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
}

// CGAL::Vector_3를 Eigen::Vector3d로 변환하는 함수
Eigen::Vector3d Point3toEigenVector(const Kernel::Point_3 &Point)
{
    Eigen::Vector3d eigen_vec(Point.x(), Point.y(), Point.z());
    // std::cout << "Converted CGAL vector [" << vec << "] to Eigen vector [" << eigen_vec.transpose() << "]" << std::endl;
    return eigen_vec;
}

// CGAL::Vector_3를 Eigen::Vector3d로 변환하는 함수
Eigen::Vector3d Vector3toEigenVector(const Kernel::Vector_3 &vec)
{
    Eigen::Vector3d eigen_vec(vec.x(), vec.y(), vec.z());
    // std::cout << "Converted CGAL vector [" << vec << "] to Eigen vector [" << eigen_vec.transpose() << "]" << std::endl;
    return eigen_vec;
}

Kernel::Point_3 EigenVectortoPoint3(const Eigen::Vector3d &vec)
{
    return Kernel::Point_3(vec.x(), vec.y(), vec.z());
}
// Vec3d와 Kernel::Point_3 간의 변환 함수
Vec3d CGALPointToVec3d(const Point_3 &p)
{
    return Vec3d(p.x(), p.y(), p.z());
}

Point_3 Vec3dToCGALPoint(const Vec3d &v)
{
    return Point_3(v.x, v.y, v.z);
}
void visualizePathPoints(const std::vector<Kernel::Point_3> &path_points, visualization_msgs::MarkerArray &marker_array, int &marker_id)
{

    for (const auto &point : path_points)
    {
        Eigen::Vector3d eigen_point = Point3toEigenVector(point);
        visualizePoint(eigen_point, marker_array, marker_id++);
    }
}

void visualizePoints(const std::vector<Surface_mesh_shortest_path::Point_3> &points, visualization_msgs::MarkerArray &marker_array, const std::string &frame_id = "base_link", const std::string &ns = "geodesic_path", float scale = 0.001)
{

    for (size_t i = 0; i < points.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // 점의 위치 설정
        marker.pose.position.x = points[i].x();
        marker.pose.position.y = points[i].y();
        marker.pose.position.z = points[i].z();

        // 점의 크기 설정
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        // 점의 색상 설정
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f; // 불투명도

        marker_array.markers.push_back(marker);
    }
}
// Tangent vectors를 시각화하는 함수
void visualizeTangentVectors(const std::vector<Eigen::Vector3d> &tangent_vectors,
                             const std::vector<Eigen::Vector3d> &points,
                             visualization_msgs::MarkerArray &marker_array)
{
    float scale_factor = 0.1;
    for (size_t i = 0; i < tangent_vectors.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "tangent_vectors";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start_point;
        start_point.x = points[i].x();
        start_point.y = points[i].y();
        start_point.z = points[i].z();
        marker.points.push_back(start_point);

        geometry_msgs::Point end_point;
        end_point.x = start_point.x + scale_factor * tangent_vectors[i].x();
        end_point.y = start_point.y + scale_factor * tangent_vectors[i].y();
        end_point.z = start_point.z + scale_factor * tangent_vectors[i].z();
        marker.points.push_back(end_point);

        marker.scale.x = 0.002;
        marker.scale.y = 0.004;
        marker.scale.z = 0.004;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }
}

// Mesh 위의 가장 가까운 점을 찾는 함수
bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &mesh)
{
    if (mesh.faces().empty())
    {
        std::cerr << "Mesh is empty, cannot build AABB tree." << std::endl;
        return false;
    }

    Tree tree(mesh.faces().begin(), mesh.faces().end(), mesh);
    tree.build();

    auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, mesh);
    face = result.first;
    location = result.second;

    if (face == Triangle_mesh::null_face())
    {
        std::cerr << "Failed to locate face for point: " << point << std::endl;
        return false;
    }

    // std::cout << "Located face: " << face << " for point: " << point <<std::endl;
    return true;
}

bool check_located_point(const Kernel::Point_3 &original_point,
                         const face_descriptor &face,
                         const Surface_mesh_shortest_path::Barycentric_coordinates &location,
                         const Triangle_mesh &mesh)
{
    // 삼각형 face의 꼭짓점들을 얻기
    auto halfedge = mesh.halfedge(face);
    auto v0 = mesh.point(mesh.source(halfedge));
    auto v1 = mesh.point(mesh.target(halfedge));
    auto v2 = mesh.point(mesh.target(mesh.next(halfedge)));

    // Barycentric coordinates를 사용하여 점을 다시 계산
    Point_3 recalculated_point = CGAL::ORIGIN + (v0 - CGAL::ORIGIN) * location[0] + (v1 - CGAL::ORIGIN) * location[1] + (v2 - CGAL::ORIGIN) * location[2];

    // 원래 점과 다시 계산된 점 사이의 거리를 계산
    auto distance = CGAL::squared_distance(original_point, recalculated_point);

    // 두 점이 거의 같은 위치에 있으면, 함수가 제대로 동작한 것으로 간주
    const double tolerance = 1e-9; // tolerance 값을 설정
    if (distance < tolerance)
    {
        std::cout << "The located point matches the original point." << std::endl;
        return true;
    }
    else
    {
        std::cerr << "Mismatch! Original point: " << original_point << ", Recalculated point: " << recalculated_point << std::endl;
        return false;
    }
}
std::vector<nrs_vision_rviz::Waypoint> convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh mesh)
{
    std::vector<nrs_vision_rviz::Waypoint> waypoints;

    for (const auto &point : points)
    {
        Point_3 cgal_point(point.x, point.y, point.z);
        face_descriptor face;
        Surface_mesh_shortest_path::Barycentric_coordinates location;

        if (!locate_face_and_point(cgal_point, face, location, mesh))
        {
            ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", point.x, point.y, point.z);
            continue;
        }

        Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);

        nrs_vision_rviz::Waypoint waypoint_msg;
        waypoint_msg.point.x = point.x;
        waypoint_msg.point.y = point.y;
        waypoint_msg.point.z = point.z;

        waypoint_msg.normal.x = -normal.x();
        waypoint_msg.normal.y = -normal.y();
        waypoint_msg.normal.z = -normal.z();

        // std::cout << "Converted point: [" << point.x << ", " << point.y << ", " << point.z << "] to waypoint with normal: ["
        //           << waypoint_msg.normal.x << ", " << waypoint_msg.normal.y << ", " << waypoint_msg.normal.z << "]" << std::endl;

        waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}

// 벡터 회전 함수 (새로운 삼각형의 법선 벡터를 사용하여 벡터 회전)
Vec3d rotateVectorToNewNormal(
    Vec3d &vec,
    const Eigen::Vector3d &old_normal,
    const Eigen::Vector3d &new_normal)
{
    Eigen::Vector3d v = Vec3dToEigen(vec);
    Eigen::Vector3d rotation_axis = old_normal.cross(new_normal);
    double angle = acos(old_normal.dot(new_normal) / (old_normal.norm() * new_normal.norm()));

    // 회전축의 길이가 0에 가까운지 확인
    if (rotation_axis.norm() < 1e-6 || std::isnan(angle))
    {
        // std::cerr << "Invalid rotation axis or angle detected. Skipping rotation." << std::endl;

        Vec3d v2 = EigenToVec3d(v);
        return v2; // 회전하지 않고 원래 벡터를 반환
    }

    Eigen::AngleAxisd rotation(angle, rotation_axis.normalized());
    Eigen::Vector3d rotated_vec = rotation * v;
    rotated_vec = rotated_vec.normalized();
    Vec3d rotated_vec2 = EigenToVec3d(rotated_vec);

    // std::cout << "Rotated vector [" << v.transpose() << "] using old normal [" << old_normal.transpose()
    //           << "] and new normal [" << new_normal.transpose() << "] to [" << rotated_vec.transpose() << "]" << std::endl;

    return rotated_vec2;
}

// p,q를 주었을 때 pq 사이의 geodesic distance, V_p,V_q를 구해주는 함수
void geodesicbasecalcuation(const Eigen::Vector3d &p, const Eigen::Vector3d &q, Eigen::Vector3d &V_p, Eigen::Vector3d &V_q, double &geodesic_distance, const Triangle_mesh &tmesh,
                            const std::vector<TriangleFace> &mesh)
{
    Kernel::Point_3 point1(p.x(), p.y(), p.z());
    Kernel::Point_3 point2(q.x(), q.y(), q.z());
    // Mesh 위의 가장 가까운 점을 찾음
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

    // p2를 source point로 추가
    Surface_mesh_shortest_path shortest_paths(tmesh);
    shortest_paths.add_source_point(face2, location2);

    // Geodesic path 상의 점들 찾기
    std::vector<Surface_mesh_shortest_path::Point_3> path_points;
    shortest_paths.shortest_path_points_to_source_points(face1, location1, std::back_inserter(path_points));
    // p0와 p1 사이의 지오데식 거리 계산
    std::pair<double, Surface_mesh_shortest_path::Source_point_iterator> result = shortest_paths.shortest_distance_to_source_points(face1, location1);
    geodesic_distance = result.first;
    // std::cout << "geodesic_distance: " << geodesic_distance << std::endl;

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

    // 일반 Cartesian 좌표계에서의 벡터를 구함
    Eigen::Vector3d point_p1(path_points[1].x(), path_points[1].y(), path_points[1].z());
    Eigen::Vector3d point_q0(path_points[path_points.size() - 2].x(), path_points[path_points.size() - 2].y(), path_points[path_points.size() - 2].z());

    // std::cout << "vp0.x: " << vp0.x() << "vp0.y: " << vp0.y() << "vp0.z: " << vp0.z() << std::endl;
    // std::cout << "vp1.x: " << vp1.x() << "vp1.y: " << vp1.y() << "vp1.z: " << vp1.z() << std::endl;
    // std::cout << "vp2.x: " << vp2.x() << "vp2.y: " << vp2.y() << "vp2.z: " << vp2.z() << std::endl;
    // std::cout << "p0.x: " << path_points[0].x() << "p0.y: " << path_points[0].y() << "p0.z: " << path_points[0].z() << std::endl;
    // std::cout << "p1.x: " << path_points[1].x() << "p1.y: " << path_points[1].y() << "p1.z: " << path_points[1].z() << std::endl;

    // std::cout << "vq0.x: " << vq0.x() << "vq0.y: " << vq0.y() << "vq0.z: " << vq0.z() << std::endl;
    // std::cout << "vq1.x: " << vq1.x() << "vq1.y: " << vq1.y() << "vq1.z: " << vq1.z() << std::endl;
    // std::cout << "vq2.x: " << vq2.x() << "vq2.y: " << vq2.y() << "vq2.z: " << vq2.z() << std::endl;
    // std::cout << "q0.x: " << path_points[path_points.size() - 2].x() << "q0.y: " << path_points[path_points.size() - 2].y() << "q0.z: " << path_points[path_points.size() - 2].z() << std::endl;
    // std::cout << "q1.x: " << path_points[path_points.size() - 1].x() << "q1.y: " << path_points[path_points.size() - 1].y() << "q1.z: " << path_points[path_points.size() - 1].z() << std::endl;

    // Barycentric 좌표계에서의 벡터를 구함

    V_p = point_p1 - p;
    V_q = q - point_q0;

    // 방향 벡터 정규화
    V_p.normalize();
    V_q.normalize();
}

// 두 벡터 사이의 각도를 계산하는 함수
double calculateAngleBetweenVectors(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2)
{
    // 벡터 간의 내적 계산
    double dot_product = vec1.dot(vec2);

    // 벡터의 크기 계산
    double magnitude_vec1 = vec1.norm();
    double magnitude_vec2 = vec2.norm();
    // 벡터 크기가 0인 경우 처리
    if (magnitude_vec1 == 0 || magnitude_vec2 == 0)
    {
        std::cerr << "Error: One of the vectors has zero length, cannot compute angle." << std::endl;
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (vec1 == vec2)
    {
        std::cout << "angle: 0.0" << std::endl;
        return 0.0;
    }

    // 내적을 벡터 크기의 곱으로 나누어 코사인 값을 얻음
    double cos_theta = dot_product / (magnitude_vec1 * magnitude_vec2);

    // acos을 이용하여 라디안 단위의 각도 계산
    double angle_rad = acos(cos_theta);

    // 라디안을 도(degree)로 변환
    double angle_deg = angle_rad * (180.0 / M_PI);
    std::cout << "angle: " << angle_deg << std::endl;

    return angle_rad;
}

Eigen::Vector3d geodesicextend(const Eigen::Vector3d &p,
                               const Eigen::Vector3d &q,
                               const Eigen::Vector3d &V_q,
                               const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh, double angle)
{
    Kernel::Point_3 point1(p.x(), p.y(), p.z());
    Kernel::Point_3 point2(q.x(), q.y(), q.z());

    // Mesh 위의 가장 가까운 점을 찾음
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
    // 현재 face의 법선 벡터를 계산
    Kernel::Vector_3 normal_vector = CGAL::Polygon_mesh_processing::compute_face_normal(face2, tmesh);
    Eigen::Vector3d normal(normal_vector.x(), normal_vector.y(), normal_vector.z());

    if (normal.norm() == 0)
    {
        throw std::runtime_error("Invalid normal vector, cannot perform rotation.");
    }

    // 회전 변환을 적용하여 새로운 벡터를 계산
    Eigen::AngleAxisd rotation(angle, normal.normalized());
    Eigen::Vector3d rotated_vector = rotation * V_q;

    return rotated_vector;
}

// 주어진 point와 direction을 투영하고 교차점을 계산하는 함수
#include <iostream>
#include <vector>
#include <limits>
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
    // face_descriptor new_face_descriptor;
    // std::cout << "face2_New Point: (" << updated_point.x << ", " << updated_point.y << ", " << updated_point.z << ")\n";
    locate_face_and_point(Vec3dToCGALPoint(updated_point), current_face_descriptor, barycentric_coords, tmesh);

    // CGAL을 이용하여 current_point의 face와 Barycentric 좌표를 찾음
    // if (!locate_face_and_point(Vec3dToCGALPoint(current_point), current_face_descriptor, barycentric_coords, tmesh))
    // {
    //     std::cerr << "Failed to locate point on mesh." << std::endl;
    //     return std::make_tuple(Vec3d{0, 0, 0}, current_direction); // 오류 처리
    // }

    // Current face's vertices
    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
    auto v_it = vertices.begin();

    Point_3 v1 = tmesh.point(*v_it++);
    Point_3 v2 = tmesh.point(*v_it++);
    Point_3 v3 = tmesh.point(*v_it);

    Vec3d v1_vec = CGALPointToVec3d(v1);
    Vec3d v2_vec = CGALPointToVec3d(v2);
    Vec3d v3_vec = CGALPointToVec3d(v3);

    // Projected point and direction are already in Barycentric coordinates
    Vec3d projected_point = current_point;
    Vec3d projected_direction = current_direction.normalize();

    // Barycentric 좌표로 변환
    Eigen::Matrix2d T;
    T << (v2_vec - v1_vec).x, (v3_vec - v1_vec).x,
        (v2_vec - v1_vec).y, (v3_vec - v1_vec).y;

    Eigen::Vector2d bary_p0 = T.inverse() * Eigen::Vector2d(projected_point.x - v1_vec.x, projected_point.y - v1_vec.y);
    Eigen::Vector2d bary_direction = T.inverse() * Eigen::Vector2d(projected_direction.x, projected_direction.y);

    // 교차점 찾기
    double t_intersect = std::numeric_limits<double>::max();
    Eigen::Vector2d bary_intersection;
    // Edge 1: b1 = 0
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

    // Edge 2: b2 = 0
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

    // Edge 3: 1 - b1 - b2 = 0
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
    // 최종 교차점 계산
    Vec3d final_point = v1_vec + bary_intersection.x() * (v2_vec - v1_vec) + bary_intersection.y() * (v3_vec - v1_vec);

    Vec3d new_direction = (final_point - current_point).normalize();

    const double epsilon = 1e-6; // 충분히 작은 값 설정

    // std::cout << "Final Point: (" << final_point.x << ", " << final_point.y << ", " << final_point.z << ")\n";
    // std::cout << "Current Point: (" << current_point.x << ", " << current_point.y << ", " << current_point.z << ")\n";

    if ((std::abs(final_point.x - current_point.x) < epsilon) &&
        (std::abs(final_point.y - current_point.y) < epsilon) &&
        (std::abs(final_point.z - current_point.z) < epsilon))
    {
        // std::cout << "new point is same with current point" << std::endl;

        // 작은 epsilon 값으로 current_point를 이동시킴
        Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
        Vec3d offset_point = current_point + epsilon_vector;

        // std::cout << "current_point: (" << current_point.x << ", " << current_point.y << ", " << current_point.z << ")\n";
        // std::cout << "offset Point: (" << offset_point.x << ", " << offset_point.y << ", " << offset_point.z << ")\n";

        // 재귀적으로 함수를 다시 호출
        auto [updated_point, updated_direction] = project_and_find_intersection(offset_point, current_direction, distance_traveled, tmesh, mesh);
        // std::cout << "result Point: (" << updated_point.x << ", " << updated_point.y << ", " << updated_point.z << ")\n";

        return std::make_tuple(updated_point, updated_direction);
    }
    else
    {
        // std::cout << "Returning final point and direction.\n";
        // std::cout << "Final Point: (" << final_point.x << ", " << final_point.y << ", " << final_point.z << ")\n";
        // std::cout << "New Direction: (" << new_direction.x << ", " << new_direction.y << ", " << new_direction.z << ")\n";
        return std::make_tuple(final_point, new_direction);
    }
}

const double EPSILON = 1e-6; // 허용 오차

// 주어진 face와 edge를 공유하는 이웃 face를 찾는 함수
face_descriptor findNeighboringFace(
    const face_descriptor &current_face,
    const Vec3d &current_point,
    const Triangle_mesh &tmesh)
{
    // std::cout << "Debug: Entered findNeighboringFace" << std::endl;

    for (auto h : halfedges_around_face(halfedge(current_face, tmesh), tmesh))
    {
        Point_3 p1 = tmesh.point(target(h, tmesh));
        Point_3 p2 = tmesh.point(source(h, tmesh));

        // current_point가 p1과 p2 사이의 edge 위에 있는지 확인
        Vec3d p1_vec = CGALPointToVec3d(p1);
        Vec3d p2_vec = CGALPointToVec3d(p2);
        Vec3d edge_vec = p2_vec - p1_vec;
        Vec3d point_vec = current_point - p1_vec;

        // std::cout << "Debug: Checking edge ("
        //           << p1.x() << ", " << p1.y() << ", " << p1.z() << ") to ("
        //           << p2.x() << ", " << p2.y() << ", " << p2.z() << ")" << std::endl;
        // std::cout << "Debug: Current point: ("
        //           << current_point.x << ", " << current_point.y << ", " << current_point.z << ")" << std::endl;

        // Cross product to check if current_point is on the line formed by the edge
        double cross_length = edge_vec.cross(point_vec).length();

        // Dot product to check if the point is along the edge
        double dot_product = point_vec.dot(edge_vec);

        // Check if current_point is within the bounds of the edge
        double point_length = point_vec.length();
        double edge_length = edge_vec.length();

        // edge 위에 있는지 확인 (벡터를 정규화하여 비교)
        if ((cross_length < EPSILON) &&
            (dot_product > 0) &&
            (point_length <= edge_length))
        {
            // std::cout << "Debug: Neighboring face found!" << std::endl;
            return face(opposite(h, tmesh), tmesh);
        }
        else
        {
            // std::cout << "Debug: This edge does not match." << std::endl;
        }
    }

    // std::cout << "Debug: Failed to locate neighboring face." << std::endl;
    return Triangle_mesh::null_face();
}

// 법선 벡터 계산 함수 (시계 방향 또는 시계 반대 방향)
Eigen::Vector3d computeFaceNormal(
    const Vec3d &v1,
    const Vec3d &v2,
    const Vec3d &v3,
    bool clockwise = true) // 기본은 시계 방향(clockwise)으로 설정
{
    Eigen::Vector3d edge1 = Vec3dToEigen(v2) - Vec3dToEigen(v1);
    Eigen::Vector3d edge2 = Vec3dToEigen(v3) - Vec3dToEigen(v1);

    Eigen::Vector3d normal;
    if (clockwise)
    {
        normal = edge1.cross(edge2); // 시계 방향일 때
    }
    else
    {
        normal = edge2.cross(edge1); // 시계 반대 방향일 때
    }

    return normal.normalized(); // 법선 벡터를 단위 벡터로 정규화
}
// 지오데식 이동 함수
Eigen::Vector3d geodesicAddVector(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &start_direction_p,
    double total_distance,
    const Eigen::Vector3d &q,
    const Triangle_mesh &tmesh,
    const std::vector<TriangleFace> &mesh)
{
    // 초기 설정
    Eigen::Vector3d V_p;
    Eigen::Vector3d V_q;
    double geodesic_distance;
    double distance_traveled = 0.0;
    Eigen::Vector3d start_direction;

    // p와 q가 동일한 경우 처리
    if (p == q)
    {
        // std::cout << "p is equal to q" << std::endl;
        start_direction = start_direction_p;
    }
    else
    {
        geodesicbasecalcuation(p, q, V_p, V_q, geodesic_distance, tmesh, mesh);
        double angle = calculateAngleBetweenVectors(V_p, start_direction_p);
        start_direction = geodesicextend(p, q, V_q, tmesh, mesh, angle);
    }

    // current_point와 direction 설정
    Vec3d current_point = EigenToVec3d(q);
    Vec3d current_direction = EigenToVec3d(start_direction).normalize();

    // 초기 face의 정보 설정
    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    // 현재 current_point가 속한 face 찾기
    if (!locate_face_and_point(Vec3dToCGALPoint(current_point), current_face_descriptor, barycentric_coords, tmesh))
    {
        std::cerr << "Failed to locate point on mesh." << std::endl;
        return Vec3dToEigen(current_point);
    }
    // std::cout << "total_distance: " << total_distance << std::endl;

    // 이동 루프: total_distance만큼 이동할 때까지 반복
    while (true)
    {

        // current_face의 정점을 얻음
        auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
        auto v_it = vertices.begin();

        Point_3 v1 = tmesh.point(*v_it++);
        Point_3 v2 = tmesh.point(*v_it++);
        Point_3 v3 = tmesh.point(*v_it);

        Vec3d v1_vec = CGALPointToVec3d(v1);
        Vec3d v2_vec = CGALPointToVec3d(v2);
        Vec3d v3_vec = CGALPointToVec3d(v3);
        // 직접 만든 함수로 old_normal 계산
        Eigen::Vector3d current_normal = computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        // current_face의 vertex 정보 출력
        std::cout << "Current Face Vertices:\n";

        for (auto v : vertices)
        {
            auto point = tmesh.point(v);
            std::cout << "Vertex: (" << point.x() << ", " << point.y() << ", " << point.z() << ")\n";
        }

        // // current_face의 normal 정보 출력
        // std::cout << "current Face: " << current_face_descriptor << " Current Face Normal: (" << current_normal.x() << ", " << current_normal.y() << ", " << current_normal.z() << ")\n";

        // // current_point와 direction 정보 출력
        // std::cout << "Current Point: (" << current_point.x << ", " << current_point.y << ", " << current_point.z << ")\n";
        // std::cout << "Current Direction: (" << current_direction.x << ", " << current_direction.y << ", " << current_direction.z << std::endl;

        // current_point를 업데이트 (project_and_find_intersection 함수 이용)

        auto [new_point, new_direction] = project_and_find_intersection(current_point, current_direction, distance_traveled, tmesh, mesh);

        // Step 3: new_normal 계산 (업데이트된 current_point가 속한 새로운 face의 normal)
        face_descriptor new_face_descriptor;

        //  locate_face_and_point를 사용하여 새로운 face를 찾음
        locate_face_and_point(Vec3dToCGALPoint(new_point), new_face_descriptor, barycentric_coords, tmesh);

        // 이전 face와 동일하면, 이웃 face를 찾아야 함
        if (new_face_descriptor == current_face_descriptor)
        {
            std::cout << "new face is same as current_face" << std::endl;
            // 새 face를 찾기 위해, current_point를 살짝 이동시킴
            Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
            Vec3d updated_point = new_point + epsilon_vector;
            // face_descriptor new_face_descriptor;
            std::cout << "face2_New Point: (" << updated_point.x << ", " << updated_point.y << ", " << updated_point.z << ")\n";
            if (!locate_face_and_point(Vec3dToCGALPoint(updated_point), new_face_descriptor, barycentric_coords, tmesh))
            {
                std::cerr << "Failed to locate new point on mesh." << std::endl;
                return Vec3dToEigen(updated_point);
            }
            std::cout << "new_face_descriptor1: " << new_face_descriptor << std::endl;
        }
        std::cout << "new_face_descriptor2: " << new_face_descriptor << std::endl;
        // new_normal 계산 (직접 만든 함수 이용)
        vertices = CGAL::vertices_around_face(tmesh.halfedge(new_face_descriptor), tmesh);
        v_it = vertices.begin();

        v1 = tmesh.point(*v_it++);
        v2 = tmesh.point(*v_it++);
        v3 = tmesh.point(*v_it);

        v1_vec = CGALPointToVec3d(v1);
        v2_vec = CGALPointToVec3d(v2);
        v3_vec = CGALPointToVec3d(v3);

        Eigen::Vector3d new_normal = computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        // new_face의 vertex 정보 출력
        // std::cout << "New Face Vertices:\n";
        // for (auto v : vertices)
        // {
        //     auto point = tmesh.point(v);
        //     std::cout << "Vertex: (" << point.x() << ", " << point.y() << ", " << point.z() << ")\n";
        // }
        // direction을 new_normal을 기준으로 회전시켜 업데이트
        new_direction = rotateVectorToNewNormal(new_direction, current_normal, new_normal);

        // // new_face의 normal 정보 출력
        // std::cout << "New Face: " << new_face_descriptor << " New Face Normal: (" << new_normal.x() << ", " << new_normal.y() << ", " << new_normal.z() << ")\n";

        // // 업데이트된 new_point와 direction 정보 출력
        // std::cout << "New Point: (" << new_point.x << ", " << new_point.y << ", " << new_point.z << ")\n";
        // std::cout << "New Direction: (" << new_direction.x << ", " << new_direction.y << ", " << new_direction.z << ")\n";
        double new_distance_traveled = sqrt((new_point.x - current_point.x) * (new_point.x - current_point.x) + (new_point.y - current_point.y) * (new_point.y - current_point.y) + (new_point.z - current_point.z) * (new_point.z - current_point.z));
        distance_traveled += (new_distance_traveled);

        // 현재 이동한 퍼센트 계산
        double percentage_traveled = (distance_traveled / total_distance) * 100.0;

        // 퍼센트 출력
        std::cout << "Progress: " << percentage_traveled << "%, Distance Traveled: " << distance_traveled << " / " << total_distance << "\n"
                  << std::endl;

        // 만약 이동 거리가 total_distance에 도달했거나 초과했다면 종료
        // 현재 face를 새로운 face로 업데이트
        current_face_descriptor = new_face_descriptor;
        current_point = new_point;
        current_direction = new_direction;

        if (distance_traveled >= total_distance)
        {
            std::cout << "Adding vector done." << std::endl;
            break;
        }
    }

    // 최종 위치 반환
    return Vec3dToEigen(current_point);
}

// 두 vertex_descriptor 간의 geodesic distance를 계산하는 함수
double computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh)
{
    // CGAL의 Point_3로 변환
    Kernel::Point_3 point0(p0.x(), p0.y(), p0.z());
    Kernel::Point_3 point1(p1.x(), p1.y(), p1.z());

    // p0와 p1의 삼각형 face와 barycentric 좌표를 찾음
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

    // Surface_mesh_shortest_path 객체를 사용하여 p1을 source point로 추가
    Surface_mesh_shortest_path shortest_paths(mesh);
    shortest_paths.add_source_point(face1, location1);

    // p0와 p1 사이의 지오데식 거리 계산
    std::pair<double, Surface_mesh_shortest_path::Source_point_iterator> result = shortest_paths.shortest_distance_to_source_points(face0, location0);

    // 거리 반환 (result.first는 지오데식 거리)
    return result.first;
}
// Geodesic interpolation parameters를 계산하는 함수
void updateInterpolationParameters(std::vector<double> &u_values, const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, bool chord_length = true)
{
    // p(i-1)과 p(i) 사이의 geodesic distance 계산
    double geodesic_distance = computeGeodesicDistance(p0, p1, mesh);

    // 필요에 따라 geodesic distance에 제곱근을 씌움
    if (!chord_length)
    {
        geodesic_distance = std::sqrt(geodesic_distance);
    }

    // u_i = u_{i-1} + d(p(i-1), p(i)) 또는 u_i = u_{i-1} + sqrt(d(p(i-1), p(i)))
    double ui = u_values.back() + geodesic_distance;
    u_values.push_back(ui);
}

// Geodesic subtraction of points in the geodesic domain
Eigen::Vector3d geodesicSubtract(
    const Eigen::Vector3d &p1,
    const Eigen::Vector3d &p2,
    const Triangle_mesh &tmesh,
    visualization_msgs::MarkerArray &marker_array, int &marker_id)
{
    Eigen::Vector3d V_p, V_q;
    double geodesic_distance;
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    geodesicbasecalcuation(p1, p2, V_p, V_q, geodesic_distance, tmesh, mesh);

    // std::cout << "direction_vector between p1 and p2 : " << direction_vector_2d << " distance bewteewn p1 and p2: " << geodesic_distance << std::endl;

    // Eigen::Vector3d pp2 = geodesicAddVector(p1, V_p, geodesic_distance, p1, tmesh, mesh);

    // std::cout << "p2: " << p2.x() << " " << p2.y() << " " << p2.z() << " pp2: " << pp2.x() << " " << pp2.y() << " " << pp2.z() << std::endl;

    return V_p * geodesic_distance;
}

// Geodesic tangent vectors를 계산하는 함수
std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array, int &marker_id)
{
    std::vector<Eigen::Vector3d> tangent_vectors;

    // 첫 번째 점과 마지막 점의 좌표를 가져옴
    std::cout << "selected_points size: " << selected_points.size() << std::endl;
    std::cout << "u_values size: " << u_values.size() << std::endl;

    if (selected_points.empty() || u_values.empty())
    {
        throw std::runtime_error("selected_points or u_values are empty!");
    }

    // selected_points에서 직접 가져옴
    Eigen::Vector3d p_first = selected_points.front();
    Eigen::Vector3d p_last = selected_points.back();

    std::cout << "First point: " << p_first.transpose() << std::endl;
    std::cout << "Last point: " << p_last.transpose() << std::endl;

    // 첫 번째 접선 벡터 정의
    if (p_first == p_last)
    {
        std::cout << "First and last points are equal, calculating tangent vector for the first point." << std::endl;

        Eigen::Vector3d p_prev = selected_points[1];
        Eigen::Vector3d p_now = selected_points[0];
        Eigen::Vector3d p_next = selected_points[selected_points.size() - 2];

        Eigen::Vector3d tangent_vector = 0.5 * (geodesicSubtract(p_prev, p_next, tmesh, marker_array, marker_id)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));

        tangent_vectors.push_back(tangent_vector);

        std::cout << "Tangent vector first: " << tangent_vector.transpose() << std::endl;
    }
    else
    {
        std::cout << "First and last points are not equal, setting first tangent vector to zero." << std::endl;
        tangent_vectors.push_back(Eigen::Vector3d::Zero()); // 첫 번째 접선 벡터는 0
    }

    // 중간 접선 벡터들 계산
    for (size_t i = 1; i < selected_points.size() - 1; ++i)
    {
        Eigen::Vector3d p_prev = selected_points[i - 1];
        Eigen::Vector3d p_now = selected_points[i];
        Eigen::Vector3d p_next = selected_points[i + 1];

        Eigen::Vector3d tangent_vector = 0.5 * (geodesicSubtract(p_prev, p_next, tmesh, marker_array, marker_id)) / (u_values[i + 1] - u_values[i - 1]);
        tangent_vectors.push_back(tangent_vector);

        std::cout << "Tangent vector " << i << ": " << tangent_vector.transpose() << std::endl;
    }

    // 마지막 접선 벡터 정의
    if (p_first == p_last)
    {
        std::cout << "Calculating tangent vector for the last point." << std::endl;

        Eigen::Vector3d p_prev = selected_points[1];
        Eigen::Vector3d p_now = selected_points[0];
        Eigen::Vector3d p_next = selected_points[selected_points.size() - 2];
        Eigen::Vector3d tangent_vector = 0.5 * (geodesicSubtract(p_prev, p_next, tmesh, marker_array, marker_id)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));
        tangent_vectors.push_back(tangent_vector);
        std::cout << "Tangent vector last: " << tangent_vector.transpose() << std::endl;
    }
    else
    {
        std::cout << "Setting last tangent vector to zero." << std::endl;
        tangent_vectors.push_back(Eigen::Vector3d::Zero()); // 마지막 접선 벡터는 0
    }

    return tangent_vectors;
}

void visualizeControlPoints(const Eigen::Vector3d &point, visualization_msgs::MarkerArray &marker_array, int &marker_id, const std::string &ns, const std::string &frame_id = "base_link")
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.007; // 점의 크기 설정
    marker.scale.y = 0.007;
    marker.scale.z = 0.007;

    marker.color.r = 1.0f; // 점의 색상 설정
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f; // 불투명도

    marker_array.markers.push_back(marker);
}

// Bézier 곡선의 제어점을 계산하는 함수
std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const std::vector<Eigen::Vector3d> &tangent_vectors,
    const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array)
{
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    std::vector<std::vector<Eigen::Vector3d>> bezier_control_points;
    int marker_id = 0; // marker_id 초기화

    // selected_points에서 직접 가져옴
    Eigen::Vector3d p_first = selected_points.front();
    Eigen::Vector3d p_last = selected_points.back();

    Eigen::Vector3d p_prev = selected_points[selected_points.size() - 2];
    Eigen::Vector3d p_now = selected_points[0];
    Eigen::Vector3d p_next = selected_points[1];

    Eigen::Vector3d tangent_prev = tangent_vectors[tangent_vectors.size() - 2];
    Eigen::Vector3d tangent_now = tangent_vectors[0];
    Eigen::Vector3d tangent_next = tangent_vectors[1];
    double distance = (u_values[1] - u_values[0]) / 3.0;
    // 첫 번째 Segment의 control point 정의
    if (p_first == p_last)
    {
        std::cout << "First and last points are equal, calculating Control Points for the first segment." << std::endl;

        Eigen::Vector3d b0 = p_now;
        Eigen::Vector3d b1 = geodesicAddVector(p_prev, tangent_now, distance * tangent_now.norm(), p_now, tmesh, mesh);
        Eigen::Vector3d b2 = geodesicAddVector(p_now, tangent_next, -distance * tangent_next.norm(), p_next, tmesh, mesh);
        Eigen::Vector3d b3 = p_next;
        bezier_control_points.push_back({b0, b1, b2, b3});
    }
    else
    {
        std::cout << "First and last points are not equal, setting first control points" << std::endl;
        double distance = (u_values[1] - u_values[0]) / 3.0;
        Eigen::Vector3d b2 = geodesicAddVector(p_now, tangent_next, -distance * tangent_next.norm(), p_next, tmesh, mesh);
        bezier_control_points.push_back({p_now, p_now, b2, p_next});
    }

    // 중간 접선 벡터들 계산
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
        Eigen::Vector3d b2 = geodesicAddVector(p_now, tangent_next, -distance * tangent_next.norm(), p_next, tmesh, mesh);
        Eigen::Vector3d b3 = p_next;
        bezier_control_points.push_back({b0, b1, b2, b3});
    }

    // 마지막 접선 벡터 정의
    if (p_first == p_last)
    {
        std::cout << "First and last points are equal, calculating Control Points for the last segment." << std::endl;

        Eigen::Vector3d b0 = p_prev;
        double distance = (u_values[0] - u_values[u_values.size() - 2]) / 3.0;
        Eigen::Vector3d b1 = geodesicAddVector(p_next, tangent_prev, distance * tangent_prev.norm(), p_prev, tmesh, mesh);
        Eigen::Vector3d b2 = geodesicAddVector(p_prev, tangent_now, -distance * tangent_now.norm(), p_now, tmesh, mesh);
        Eigen::Vector3d b3 = p_now;
        bezier_control_points.push_back({b0, b1, b2, b3});
    }
    else
    {
        std::cout << "First and last points are not equal, setting last control points" << std::endl;
        double distance = (u_values[u_values.size() - 1] - u_values[u_values.size() - 2]) / 3.0;
        Eigen::Vector3d b1 = geodesicAddVector(selected_points[selected_points.size() - 3], tangent_vectors[tangent_vectors.size() - 2], distance * tangent_prev.norm(), selected_points[selected_points.size() - 2], tmesh, mesh);

        bezier_control_points.push_back({selected_points[selected_points.size() - 2], b1, selected_points[selected_points.size() - 1], selected_points[selected_points.size() - 1]});
    }

    for (int i = 0; i < bezier_control_points.size(); i++)
    {
        // RViz에 b1과 b2를 시각화
        visualizeControlPoints(bezier_control_points[i][1], marker_array, marker_id, "control_point_b1");
        visualizeControlPoints(bezier_control_points[i][2], marker_array, marker_id, "control_point_b2");
        std::cout << "Computed Bezier control points for segment " << i + 1 << ":\n"
                  << "B0: " << bezier_control_points[i][0].transpose() << "\n"
                  << "B1: " << bezier_control_points[i][1].transpose() << "\n"
                  << "B2: " << bezier_control_points[i][2].transpose() << "\n"
                  << "B3: " << bezier_control_points[i][3].transpose() << std::endl;
    }

    return bezier_control_points;
}

// Bézier 곡선을 Bernstein 형식으로 계산하는 함수
std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(
    const std::vector<Eigen::Vector3d> &control_points,
    const Triangle_mesh &tmesh,
    int steps,
    visualization_msgs::MarkerArray &marker_array,
    int &marker_id)
{
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    std::vector<Eigen::Vector3d> curve_points;
    Eigen::Vector3d V_p1, V_q1;
    double geodesic_distance;
    curve_points.reserve(steps + 1);

    for (int i = 0; i < steps; ++i)
    {

        std::cout << "now step: " << i << std::endl;
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d p1 = (1 - t) * (1 - t) * (1 - t) * control_points[0];
        Eigen::Vector3d p2 = 3 * (1 - t) * (1 - t) * t * control_points[1];
        Eigen::Vector3d p3 = 3 * (1 - t) * t * t * control_points[2];
        Eigen::Vector3d p4 = t * t * t * control_points[3];
        std::cout << "p1.x: " << p1.x() << "p1.y: " << p1.y() << "p1.z: " << p1.z() << std::endl;
        std::cout << "p2.x: " << p2.x() << "p2.y: " << p2.y() << "p2.z: " << p2.z() << std::endl;
        std::cout << "p3.x: " << p3.x() << "p3.y: " << p3.y() << "p3.z: " << p3.z() << std::endl;
        std::cout << "p4.x: " << p4.x() << "p4.y: " << p4.y() << "p3.z: " << p3.z() << std::endl;

        Eigen::Vector3d v1 = geodesicSubtract(p1, p2, tmesh, marker_array, marker_id);
        Eigen::Vector3d v2 = geodesicSubtract(p1, p4, tmesh, marker_array, marker_id);
        Eigen::Vector3d v3 = geodesicSubtract(p1, p3, tmesh, marker_array, marker_id);

        std::cout << "v1.x: " << v1.x() << "v1.y: " << v1.y() << "v1.z: " << v1.z() << std::endl;
        std::cout << "v2.x: " << v2.x() << "v2.y: " << v2.y() << "v2.z: " << v2.z() << std::endl;
        std::cout << "v3.x: " << v3.x() << "v3.y: " << v3.y() << "v3.z: " << v3.z() << std::endl;

        double angle1 = calculateAngleBetweenVectors(v1, v2);

        Eigen::Vector3d q1 = geodesicAddVector(p1, v1, v1.norm(), p1, tmesh, mesh); // p1에서 v1만큼 떨어진 점 q1
        std::cout << "calculating q1 done." << std::endl;
        std::cout << "q1.x: " << q1.x() << "q1.y: " << q1.y() << "q1.z: " << q1.z() << std::endl;

        geodesicbasecalcuation(p1, q1, V_p1, V_q1, geodesic_distance, tmesh, mesh);
        Eigen::Vector3d v2_q1 = v2.norm() * geodesicextend(p1, q1, V_q1, tmesh, mesh, angle1); // q1에서의 v2벡터
        std::cout << "calculating v2_q1 done." << std::endl;
        std::cout << "v2_q1.x: " << v2_q1.x() << "v2_q1.y: " << v2_q1.y() << "v2_q1.z: " << v2_q1.z() << std::endl;

        double angle2 = calculateAngleBetweenVectors(v1, v3);
        Eigen::Vector3d v3_q1 = v3.norm() * geodesicextend(p1, q1, V_q1, tmesh, mesh, angle2); // q1에서의 v3벡터
        std::cout << "calculating v3_q1 done." << std::endl;
        std::cout << "v3_q1.x: " << v3_q1.x() << "v3_q1.y: " << v3_q1.y() << "v3_q1.z: " << v3_q1.z() << std::endl;

        double angle3 = calculateAngleBetweenVectors(v2_q1, v3_q1);                       // q1에서의 v2벡터와 v3벡터 사이의 각도
        Eigen::Vector3d q2 = geodesicAddVector(q1, v2_q1, v2_q1.norm(), q1, tmesh, mesh); // q1 + q1에서의 v2벡터 = q2
        std::cout << "calculating q2 done." << std::endl;
        std::cout << "q2.x: " << q2.x() << "q2.y: " << q2.y() << "q2.z: " << q2.z() << std::endl;

        Eigen::Vector3d v3_q2 = v3_q1.norm() * geodesicextend(q1, q2, v3_q1, tmesh, mesh, angle3); // q2에서의 v3벡터
        std::cout << "calculating v3_q2 done." << std::endl;
        std::cout << "v3_q2.x: " << v3_q2.x() << "v3_q2.y: " << v3_q2.y() << "v3_q2.z: " << v3_q2.z() << std::endl;

        Eigen::Vector3d point = v3_q2;

        curve_points.push_back(point);
        std::cout << "curve_points[" << i << "]: " << curve_points[i].x() << "|" << curve_points[i].y() << "|" << curve_points[i].z() << std::endl;
    }

    return curve_points;
}

// Hermite 스플라인을 계산하는 함수 (여러 Bézier 곡선을 연결)
void generate_Hermite_Spline_path(
    const std::vector<std::vector<Eigen::Vector3d>> &bezier_control_points,
    int steps, visualization_msgs::MarkerArray &marker_array, int marker_id)
{
    std::vector<Eigen::Vector3d> hermite_spline;

    for (const auto &control_points : bezier_control_points)
    {
        // 각 Bézier 곡선을 Bernstein 형식으로 계산
        std::vector<Eigen::Vector3d> curve_points = computeGeodesicBezierCurvePoints(control_points, mesh, steps, marker_array, marker_id);
        hermite_spline.insert(hermite_spline.end(), curve_points.begin(), curve_points.end());
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
    waypoints_msg.waypoints = convertToWaypoints(path_points, mesh);
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
        if (mesh.add_face(index_to_vertex[t[0]], index_to_vertex[t[1]], index_to_vertex[t[2]]) == Triangle_mesh::null_face())
        {
            ROS_ERROR("Failed to add face.");
            return false;
        }
    }
    ROS_INFO("Successfully read STL file.");
    return true;
}

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    std::cout << "-----------------------------------new waypoints comming----------------------------------------------" << std::endl;

    // clicked_point를 Kernel::Point_3로 변환
    Kernel::Point_3 clicked_point(msg->point.x, msg->point.y, msg->point.z);

    // Mesh 위의 face와 barycentric 좌표를 찾음
    face_descriptor face;
    Surface_mesh_shortest_path::Barycentric_coordinates location;

    if (!locate_face_and_point(clicked_point, face, location, mesh))
    {
        std::cerr << "Failed to locate the clicked point on the mesh." << std::endl;
        return; // 만약 실패하면 아무것도 하지 않음
    }

    // barycentric 좌표를 사용하여 클릭한 점을 정확한 메쉬 위의 점으로 변환
    // face의 세 vertex를 가져옴
    Kernel::Point_3 v1, v2, v3;
    int i = 0;

    for (auto v : vertices_around_face(mesh.halfedge(face), mesh))
    {
        if (i == 0)
            v1 = mesh.point(v);
        else if (i == 1)
            v2 = mesh.point(v);
        else if (i == 2)
            v3 = mesh.point(v);

        ++i;
    }

    // 투영된 점을 계산
    Kernel::Point_3 projected_point = CGAL::barycenter(v1, location[0], v2, location[1], v3, location[2]);

    // Eigen::Vector3d로 변환
    Eigen::Vector3d projected_point_eigen(projected_point.x(), projected_point.y(), projected_point.z());

    // 투영된 점을 사용하여 처리
    clicked_points.push_back(msg->point);             // 원본 clicked_point를 유지
    selected_points.push_back(projected_point_eigen); // Mesh 위의 점을 저장

    new_waypoints = true; // 새로운 waypoint가 추가될 때 플래그를 true로 설정
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hermite_to_bezier");
    ros::NodeHandle nh;

    // Marker를 publish할 publisher 생성
    marker_pub2 = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array_hermite", 10);
    ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10);
    int marker_id = 0;
    std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/lid_wrap.stl";
    std::ifstream input(mesh_file_path, std::ios::binary);
    read_stl_file(input, mesh);

    tree = new Tree(mesh.faces().begin(), mesh.faces().end(), mesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(mesh);

    ros::Rate r(30);

    while (ros::ok())
    {
        ros::spinOnce();
        if (new_waypoints)
        {
            visualization_msgs::MarkerArray marker_array; // 모든 마커들을 담을 MarkerArray 생성

            // u_values 업데이트
            if (selected_points.size() > 1)
            {
                updateInterpolationParameters(u_values, selected_points[selected_points.size() - 2], selected_points[-1], true);
            }
            if (selected_points.size() > 2 && u_values.size() > 2)
            {
                // Geodesic tangent vectors 계산
                tangent_vectors = calculateGeodesicTangentVectors(selected_points, u_values, mesh, marker_array, marker_id);

                // Tangent vectors 시각화
                // visualizeTangentVectors(tangent_vectors, selected_points, marker_array);

                // 터미널에 출력
                std::cout << "selected_points:" << std::endl;
                for (const auto &point : selected_points)
                {
                    std::cout << "points: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
                }

                std::cout << "u_values:" << std::endl;
                for (size_t i = 0; i < u_values.size(); ++i)
                {
                    std::cout << "u_" << i << ": " << u_values[i] << std::endl;
                }

                std::cout << "Tangent Vectors:" << std::endl;
                for (size_t i = 0; i < tangent_vectors.size(); ++i)
                {
                    std::cout << "Tangent Vector " << i << ": " << tangent_vectors[i].transpose() << std::endl;
                }

                // Bézier 곡선의 제어점을 계산
                std::vector<std::vector<Eigen::Vector3d>> bezier_control_points = computeBezierControlPoints(selected_points, u_values, tangent_vectors, mesh, marker_array);

                // Hermite 스플라인을 Bernstein 형식으로 계산

                generate_Hermite_Spline_path(bezier_control_points, 3, marker_array, 1);
                marker_pub2.publish(marker_array);
            }
            waypoints_pub.publish(waypoints_msg); // 경로 생성 후 퍼블리시
            new_waypoints = false;                // 플래그 초기화
        }

        r.sleep();
    }

    delete tree;
    delete shortest_paths;
    return 0;
}
