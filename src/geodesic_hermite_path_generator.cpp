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
Triangle_mesh tmesh;
Tree *tree;
Surface_mesh_shortest_path *shortest_paths;
nrs_vision_rviz::Waypoints waypoints_msg;
ros::Publisher waypoints_pub;
std::vector<Eigen::Vector3d> selected_points; // 선택한 점들
std::vector<double> u_values = {0.0};         // u_0 = 0으로 초기화
std::vector<Eigen::Vector3d> tangent_vectors; // 선택한 점들의 tangent vectors
int steps_per_segment = 20;                   // 각 Bézier 곡선을 몇 단계로 세분화할지 결정

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

    // std::cout << "Located face: " << face << " for point: " << point << std::endl;
    return true;
}

std::vector<nrs_vision_rviz::Waypoint> convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh tmesh)
{
    std::vector<nrs_vision_rviz::Waypoint> waypoints;

    for (const auto &point : points)
    {
        Point_3 cgal_point(point.x, point.y, point.z);
        face_descriptor face;
        Surface_mesh_shortest_path::Barycentric_coordinates location;

        if (!locate_face_and_point(cgal_point, face, location, tmesh))
        {
            ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", point.x, point.y, point.z);
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

        // std::cout << "Converted point: [" << point.x << ", " << point.y << ", " << point.z << "] to waypoint with normal: ["
        //           << waypoint_msg.normal.x << ", " << waypoint_msg.normal.y << ", " << waypoint_msg.normal.z << "]" << std::endl;

        waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}

// face에서 가장 가까운 vertex를 찾는 함수
vertex_descriptor get_closest_vertex_on_face(const face_descriptor &face, const Kernel::Point_3 &point, const Triangle_mesh &mesh)
{
    vertex_descriptor closest_vertex;
    double min_distance = std::numeric_limits<double>::max();

    for (auto v : vertices_around_face(mesh.halfedge(face), mesh))
    {
        double distance = std::sqrt(CGAL::squared_distance(point, mesh.point(v)));
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_vertex = v;
        }
    }

    std::cout << "Closest vertex on face: " << face << " for point: " << point << " is vertex: " << closest_vertex << " with distance: " << min_distance << std::endl;

    return closest_vertex;
}

// CGAL::Vector_3를 Eigen::Vector3d로 변환하는 함수
Eigen::Vector3d Vector3toEigenVector(const Kernel::Vector_3 &vec)
{
    Eigen::Vector3d eigen_vec(vec.x(), vec.y(), vec.z());
    // std::cout << "Converted CGAL vector [" << vec << "] to Eigen vector [" << eigen_vec.transpose() << "]" << std::endl;
    return eigen_vec;
}

// 벡터 회전 함수 (새로운 삼각형의 법선 벡터를 사용하여 벡터 회전)
Eigen::Vector3d rotateVectorToNewNormal(
    const Eigen::Vector3d &v,
    const Eigen::Vector3d &old_normal,
    const Eigen::Vector3d &new_normal)
{
    Eigen::Vector3d rotation_axis = old_normal.cross(new_normal);
    double angle = acos(old_normal.dot(new_normal) / (old_normal.norm() * new_normal.norm()));

    // 회전축의 길이가 0에 가까운지 확인
    if (rotation_axis.norm() < 1e-6 || std::isnan(angle))
    {
        // std::cerr << "Invalid rotation axis or angle detected. Skipping rotation." << std::endl;
        return v; // 회전하지 않고 원래 벡터를 반환
    }

    Eigen::AngleAxisd rotation(angle, rotation_axis.normalized());
    Eigen::Vector3d rotated_vec = rotation * v;

    std::cout << "Rotated vector [" << v.transpose() << "] using old normal [" << old_normal.transpose()
              << "] and new normal [" << new_normal.transpose() << "] to [" << rotated_vec.transpose() << "]" << std::endl;

    return rotated_vec;
}

// Barycentric 좌표를 3D 좌표로 변환하는 함수
Eigen::Vector3d convertBarycentricTo3D(const Triangle_mesh &mesh, const face_descriptor &face, const Surface_mesh_shortest_path::Barycentric_coordinates &barycentric_coords)
{
    // face로부터 세 개의 vertex 가져오기
    auto halfedge = mesh.halfedge(face);

    auto v0 = mesh.target(halfedge);
    auto v1 = mesh.target(mesh.next(halfedge));
    auto v2 = mesh.target(mesh.next(mesh.next(halfedge)));

    // 각각의 vertex 위치
    const Kernel::Point_3 &p0 = mesh.point(v0);
    const Kernel::Point_3 &p1 = mesh.point(v1);
    const Kernel::Point_3 &p2 = mesh.point(v2);

    // Barycentric 좌표를 사용하여 실제 3D 좌표 계산
    Kernel::Point_3 point_3d = CGAL::ORIGIN +
                               barycentric_coords[0] * (p0 - CGAL::ORIGIN) +
                               barycentric_coords[1] * (p1 - CGAL::ORIGIN) +
                               barycentric_coords[2] * (p2 - CGAL::ORIGIN);

    return Eigen::Vector3d(point_3d.x(), point_3d.y(), point_3d.z());
}

// 벡터를 지오데식 도메인에서 더하는 함수
// 벡터를 지오데식 도메인에서 더하는 함수
Eigen::Vector3d geodesicAddVector(
    const Triangle_mesh &mesh,
    const Kernel::Point_3 &start_point,
    const Eigen::Vector3d &start_direction,
    double total_distance, visualization_msgs::MarkerArray &marker_array, int &marker_id) // marker_id를 참조로 받아와서 사용
{
    face_descriptor current_face;
    Surface_mesh_shortest_path::Barycentric_coordinates current_location;

    if (!locate_face_and_point(start_point, current_face, current_location, mesh))
    {
        std::cerr << "Failed to locate point on mesh." << std::endl;
        return Eigen::Vector3d::Zero();
    }

    double distance_traveled = 0.0;
    Kernel::Point_3 current_point = start_point;

    Kernel::Vector_3 old_normal_cgal = CGAL::Polygon_mesh_processing::compute_face_normal(current_face, mesh);
    Eigen::Vector3d old_normal = Vector3toEigenVector(old_normal_cgal);
    Eigen::Vector3d direction = start_direction;

    std::cout << "Starting geodesic add vector with start_point: " << start_point << ", direction: ["
              << direction.transpose() << "], and total_distance: " << total_distance << std::endl;
    Kernel::Point_3 next_point = current_point + Kernel::Vector_3(direction.x(), direction.y(), direction.z()) * 0.001;

    face_descriptor next_face;
    Surface_mesh_shortest_path::Barycentric_coordinates next_location;
    if (!locate_face_and_point(next_point, next_face, next_location, mesh))
    {
        std::cerr << "Failed to locate next point on mesh." << std::endl;
    }

    Surface_mesh_shortest_path shortest_paths(mesh);
    shortest_paths.add_source_point(next_face, next_location);

    std::vector<Surface_mesh_shortest_path::Point_3> path_points;
    shortest_paths.shortest_path_points_to_source_points(current_face, current_location, std::back_inserter(path_points));
    Kernel::Point_3 edge_intersect_point;
    bool edge_intersect_found = false;
    for (size_t i = 1; i < path_points.size(); ++i)
    {

        edge_intersect_point = path_points[i];
        for (auto halfedge : halfedges_around_face(mesh.halfedge(next_face), mesh))
        {
            auto v1 = mesh.point(mesh.source(halfedge));
            auto v2 = mesh.point(mesh.target(halfedge));

            // Check if the point is on the edge
            if (CGAL::collinear(v1, v2, edge_intersect_point))
            {
                edge_intersect_found = true;
                break;
            }
        }
    }
    next_point = edge_intersect_point;
    direction = Eigen::Vector3d(
                    edge_intersect_point.x() - start_point.x(),
                    edge_intersect_point.y() - start_point.y(),
                    edge_intersect_point.z() - start_point.z())
                    .normalized();

    while (distance_traveled < total_distance)
    {
        // 엣지와의 교차점을 찾기 위해 방향 벡터와 삼각형 엣지의 교차점을 계산
        Kernel::Point_3 edge_intersect_point;
        bool edge_intersect_found = false;
        double min_t = std::numeric_limits<double>::max();

        for (auto halfedge : halfedges_around_face(mesh.halfedge(current_face), mesh))
        {
            auto v1 = mesh.point(mesh.source(halfedge));
            auto v2 = mesh.point(mesh.target(halfedge));

            Eigen::Vector3d edge_vector(v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z());
            Eigen::Vector3d start_to_v1(v1.x() - current_point.x(), v1.y() - current_point.y(), v1.z() - current_point.z());

            // 방향 벡터와 엣지 벡터 간의 교차점 계산
            double t = (start_to_v1.cross(edge_vector)).norm() / (direction.cross(edge_vector)).norm();

            if (t >= 0 && t < min_t)
            {
                min_t = t;
                edge_intersect_point = current_point + Kernel::Vector_3(direction.x(), direction.y(), direction.z()) * t;
                edge_intersect_found = true;
            }
        }
        // visualizePoint(Point3toEigenVector(edge_intersect_point),marker_array,marker_id++);

        if (!edge_intersect_found)
        {
            std::cerr << "Failed to find an edge intersection point." << std::endl;
            break;
        }

        // 거리를 업데이트
        double segment_distance = std::sqrt(CGAL::squared_distance(current_point, edge_intersect_point));
        distance_traveled += segment_distance;

        if (distance_traveled >= total_distance)
        {
            break;
        }

        // 방향 업데이트
        direction = Eigen::Vector3d(
                        edge_intersect_point.x() - current_point.x(),
                        edge_intersect_point.y() - current_point.y(),
                        edge_intersect_point.z() - current_point.z())
                        .normalized();

        // 새로운 normal을 계산하고 방향 회전
        Kernel::Vector_3 new_normal_cgal = CGAL::Polygon_mesh_processing::compute_face_normal(current_face, mesh);
        Eigen::Vector3d new_normal = Vector3toEigenVector(new_normal_cgal);

        direction = rotateVectorToNewNormal(direction, old_normal, new_normal);
        old_normal = new_normal;

        // current_point를 업데이트하여 다음 루프에서 사용할 수 있도록 설정
        current_point = edge_intersect_point;

        // 시각화를 위해 마커 추가
        // visualizePoint(Point3toEigenVector(current_point), marker_array, marker_id++);
        // visualizeDirectionArrow(Point3toEigenVector(current_point), direction, marker_array, marker_id++);
    }

    std::cout << "End point after geodesic addition: " << current_point << std::endl;
    return Eigen::Vector3d(current_point.x(), current_point.y(), current_point.z());
}

Eigen::Vector3d geodesicAdd(const Triangle_mesh &mesh,
                            const Eigen::Vector3d &pointA,
                            const Eigen::Vector3d &pointB,
                            double t,
                            visualization_msgs::MarkerArray &marker_array,
                            int &marker_id)
{
    // pointA에서 pointB로 가는 지오데식 경로를 따라 t만큼 이동한 점을 찾는다.
    Eigen::Vector3d direction = (pointB - pointA).normalized();
    double total_distance = (pointB - pointA).norm() * t;

    return geodesicAddVector(mesh, Kernel::Point_3(pointA.x(), pointA.y(), pointA.z()), direction, total_distance, marker_array, marker_id);
}

// 클릭한 점과 가장 가까운 vertex_descriptor를 찾는 함수
vertex_descriptor findClosestVertex(const geometry_msgs::Point &point)
{
    Kernel::Point_3 query(point.x, point.y, point.z);

    Tree::Point_and_primitive_id closest = tree->closest_point_and_primitive(query);
    Point_3 closest_point = closest.first;
    face_descriptor closest_face = closest.second;

    if (closest_face == Triangle_mesh::null_face())
    {
        std::cout << "Invalid closest face for point: " << query << std::endl;
        return Triangle_mesh::null_vertex();
    }

    vertex_descriptor closest_vertex = Triangle_mesh::null_vertex();
    double min_distance = std::numeric_limits<double>::max();

    for (auto v : vertices_around_face(tmesh.halfedge(closest_face), tmesh))
    {
        double distance = CGAL::squared_distance(tmesh.point(v), closest_point);

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_vertex = v;
        }
    }

    if (!tmesh.is_valid(closest_vertex))
    {
        std::cout << "Invalid vertex descriptor for face: " << closest_face << " and point: " << query << std::endl;
    }

    std::cout << "Closest vertex to point: " << query << " is vertex: " << closest_vertex << std::endl;
    return closest_vertex;
}
// 두 점 사이의 유클리드 거리를 계산하는 함수
double computeEuclideanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
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
    double geodesic_distance = computeGeodesicDistance(p0, p1, tmesh);

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
    const Eigen::Vector3d &p1, // Eigen::Vector3d로 수정
    const Eigen::Vector3d &p2, // Eigen::Vector3d로 수정
    const Triangle_mesh &mesh,
    visualization_msgs::MarkerArray &marker_array)
{
    // CGAL::Point_3로 변환
    Kernel::Point_3 point1(p1.x(), p1.y(), p1.z());
    Kernel::Point_3 point2(p2.x(), p2.y(), p2.z());

    // Mesh 위의 가장 가까운 점을 찾음
    face_descriptor face1, face2;
    Surface_mesh_shortest_path::Barycentric_coordinates location1, location2;

    if (!locate_face_and_point(point1, face1, location1, mesh))
    {
        throw std::runtime_error("Failed to locate point1 on mesh.");
    }
    if (!locate_face_and_point(point2, face2, location2, mesh))
    {
        throw std::runtime_error("Failed to locate point2 on mesh.");
    }

    // p2를 source point로 추가
    Surface_mesh_shortest_path shortest_paths(mesh);
    shortest_paths.add_source_point(face2, location2);

    // Geodesic path 상의 점들 찾기
    std::vector<Surface_mesh_shortest_path::Point_3> path_points;
    shortest_paths.shortest_path_points_to_source_points(face1, location1, std::back_inserter(path_points));

    if (path_points.size() < 2)
    {
        throw std::runtime_error("Geodesic path does not contain enough points.");
    }
    std::cout << "path_points[0]: " << path_points[0] << "path_points[1]: " << path_points[1] << std::endl;

    // Path points 전체 시각화
    // visualizePoints(path_points, marker_array, "base_link", "subtract_path");

    // p1에서 경로 상의 첫 번째 점까지의 벡터 계산 및 정규화 (unit vector)
    Kernel::Vector_3 direction_vector = path_points[1] - path_points[0];
    Eigen::Vector3d unit_vector(direction_vector.x(), direction_vector.y(), direction_vector.z());
    unit_vector.normalize(); // Unit vector로 정규화
    std::cout << "tangent unit vector x: " << unit_vector.x() << "tangent unit vector y: " << unit_vector.y() << "tangent unit vector z: " << unit_vector.z() << std::endl;

    // Geodesic distance 계산
    double geodesic_distance = computeGeodesicDistance(p1, p2, tmesh);
    std::cout << "tangent vector size: " << geodesic_distance << std::endl;
    // Unit vector에 geodesic distance를 곱하여 최종 결과 계산
    return unit_vector * geodesic_distance;
}
// Geodesic tangent vectors를 계산하는 함수
std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(
    const std::vector<Eigen::Vector3d> &selected_points,
    const std::vector<double> &u_values,
    const Triangle_mesh &mesh, visualization_msgs::MarkerArray &marker_array)
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

        Eigen::Vector3d p1 = selected_points[1];
        Eigen::Vector3d p_last_prev = selected_points[selected_points.size() - 2];

        Eigen::Vector3d tangent_vector_first = 0.5 * (geodesicSubtract(p1, p_last_prev, mesh, marker_array)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));
        tangent_vectors.push_back(tangent_vector_first);

        std::cout << "Tangent vector first: " << tangent_vector_first.transpose() << std::endl;
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
        Eigen::Vector3d p_next = selected_points[i + 1];

        Eigen::Vector3d tangent_vector = 0.5 * (geodesicSubtract(p_prev, p_next, mesh, marker_array)) / (u_values[i + 1] - u_values[i - 1]);
        tangent_vectors.push_back(tangent_vector);

        std::cout << "Tangent vector " << i << ": " << tangent_vector.transpose() << std::endl;
    }

    // 마지막 접선 벡터 정의
    if (p_first == p_last)
    {
        std::cout << "Calculating tangent vector for the last point." << std::endl;

        Eigen::Vector3d p1 = selected_points[1];
        Eigen::Vector3d p_last_prev = selected_points[selected_points.size() - 2];

        Eigen::Vector3d tangent_vector_last = 0.5 * (geodesicSubtract(p1, p_last_prev, mesh, marker_array)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));
        tangent_vectors.push_back(tangent_vector_last);

        std::cout << "Tangent vector last: " << tangent_vector_last.transpose() << std::endl;
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
    const Triangle_mesh &mesh, visualization_msgs::MarkerArray &marker_array)
{
    std::vector<std::vector<Eigen::Vector3d>> bezier_control_points;
    int marker_id = 0; // marker_id 초기화
    for (size_t i = 0; i < selected_points.size() - 1; ++i)
    {
        Eigen::Vector3d p_i = selected_points[i];
        Eigen::Vector3d p_i1 = selected_points[i + 1];

        const Eigen::Vector3d &tangent_vector_i = tangent_vectors[i];
        const Eigen::Vector3d &tangent_vector_i1 = tangent_vectors[i + 1];

        double distance = ((u_values[i + 1] - u_values[i]) / 3.0) * tangent_vector_i.norm();
        Eigen::Vector3d b0 = p_i;
        // Eigen::Vector3d b1 = geodesicAddVector(mesh, Kernel::Point_3(p_i.x(), p_i.y(), p_i.z()), tangent_vector_i.normalized(), distance);
        Eigen::Vector3d b1 = geodesicAddVector(mesh, Kernel::Point_3(p_i.x(), p_i.y(), p_i.z()), tangent_vector_i.normalized(), distance, marker_array, marker_id);

        double distance2 = ((u_values[i + 1] - u_values[i]) / 3.0) * tangent_vector_i1.norm();
        // Eigen::Vector3d b2 = geodesicAddVector(mesh, Kernel::Point_3(p_i1.x(), p_i1.y(), p_i1.z()), -tangent_vector_i1.normalized(), distance2);
        Eigen::Vector3d b2 = geodesicAddVector(mesh, Kernel::Point_3(p_i1.x(), p_i1.y(), p_i1.z()), -tangent_vector_i1.normalized(), distance2, marker_array, marker_id);

        Eigen::Vector3d b3 = p_i1;
        bezier_control_points.push_back({b0, b1, b2, b3});
        // RViz에 b1과 b2를 시각화
        visualizeControlPoints(b1, marker_array, marker_id, "control_point_b1");
        visualizeControlPoints(b2, marker_array, marker_id, "control_point_b2");
        std::cout << "Computed Bezier control points for segment " << i << ":\n"
                  << "B0: " << b0.transpose() << "\n"
                  << "B1: " << b1.transpose() << "\n"
                  << "B2: " << b2.transpose() << "\n"
                  << "B3: " << b3.transpose() << std::endl;
    }

    return bezier_control_points;
}

// Bézier 곡선을 Bernstein 형식으로 계산하는 함수
std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(
    const std::vector<Eigen::Vector3d> &control_points,
    const Triangle_mesh &mesh,
    int steps,
    visualization_msgs::MarkerArray &marker_array,
    int &marker_id)
{
    std::vector<Eigen::Vector3d> curve_points;
    curve_points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i)
    {
        double t = static_cast<double>(i) / steps;

        // 지오데식 덧셈을 사용하여 Bézier 곡선의 한 점 계산
        Eigen::Vector3d point = geodesicAdd(mesh, control_points[0], control_points[1], t, marker_array, marker_id);
        point = geodesicAdd(mesh, point, control_points[2], t, marker_array, marker_id);
        point = geodesicAdd(mesh, point, control_points[3], t, marker_array, marker_id);

        curve_points.push_back(point);
    }

    return curve_points;
}

// Hermite 스플라인을 계산하는 함수 (여러 Bézier 곡선을 연결)
void generate_Hermite_Spline_path(
    const std::vector<std::vector<Eigen::Vector3d>> &bezier_control_points,
    int steps_per_segment, int steps, visualization_msgs::MarkerArray &marker_array, int marker_id)
{
    std::vector<Eigen::Vector3d> hermite_spline;

    for (const auto &control_points : bezier_control_points)
    {
        // 각 Bézier 곡선을 Bernstein 형식으로 계산
        std::vector<Eigen::Vector3d> curve_points = computeGeodesicBezierCurvePoints(control_points, tmesh, steps, marker_array, marker_id);
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
    waypoints_msg.waypoints = convertToWaypoints(path_points, tmesh);
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

    if (!locate_face_and_point(clicked_point, face, location, tmesh))
    {
        std::cerr << "Failed to locate the clicked point on the mesh." << std::endl;
        return; // 만약 실패하면 아무것도 하지 않음
    }

    // barycentric 좌표를 사용하여 클릭한 점을 정확한 메쉬 위의 점으로 변환
    // face의 세 vertex를 가져옴
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
    ros::Publisher marker_pub2 = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array_hermite", 10);
    ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10);

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
                tangent_vectors = calculateGeodesicTangentVectors(selected_points, u_values, tmesh, marker_array);

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
                std::vector<std::vector<Eigen::Vector3d>> bezier_control_points = computeBezierControlPoints(selected_points, u_values, tangent_vectors, tmesh, marker_array);

                // Hermite 스플라인을 Bernstein 형식으로 계산

                generate_Hermite_Spline_path(bezier_control_points, steps_per_segment, 20, marker_array, 1);
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
