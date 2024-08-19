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

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

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

void visualizePathPoints(const std::vector<Kernel::Point_3> &path_points,visualization_msgs::MarkerArray &marker_array, int &marker_id)
{

    for (const auto &point : path_points)
    {
        Eigen::Vector3d eigen_point = Point3toEigenVector(point);
        visualizePoint(eigen_point, marker_array, marker_id++);
    }
}

void visualizePoints(const std::vector<Surface_mesh_shortest_path::Point_3> &points,visualization_msgs::MarkerArray &marker_array, const std::string &frame_id = "base_link", const std::string &ns = "geodesic_path", float scale = 0.001)
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

// 벡터를 지오데식 도메인에서 더하는 함수
Eigen::Vector3d geodesicAddVector(
    const Triangle_mesh &mesh,
    const Kernel::Point_3 &start_point,
    const Eigen::Vector3d &start_direction,
    double total_distance,
    visualization_msgs::MarkerArray marker_array,
    int &marker_id) // marker_id를 참조로 받아와서 사용
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

    while (distance_traveled < total_distance)
    {
        Kernel::Point_3 next_point = current_point + Kernel::Vector_3(direction.x(), direction.y(), direction.z()) * 0.001;

        face_descriptor next_face;
        Surface_mesh_shortest_path::Barycentric_coordinates next_location;
        if (!locate_face_and_point(next_point, next_face, next_location, mesh))
        {
            std::cerr << "Failed to locate next point on mesh." << std::endl;
            break;
        }

        Surface_mesh_shortest_path shortest_paths(mesh);
        shortest_paths.add_source_point(next_face, next_location);

        std::vector<Surface_mesh_shortest_path::Point_3> path_points;
        shortest_paths.shortest_path_points_to_source_points(current_face, current_location, std::back_inserter(path_points));

        if (path_points.size() > 1)
        {
            Kernel::Point_3 path_point = path_points[1];
            double segment_distance = std::sqrt(CGAL::squared_distance(current_point, path_point));

            // 전체 path_points를 시각화
            visualizePathPoints(path_points, marker_array, marker_id);
            if (distance_traveled + segment_distance >= total_distance)
            {
                double remaining_distance = total_distance - distance_traveled;
                Kernel::Point_3 final_point = current_point + Kernel::Vector_3(direction.x(), direction.y(), direction.z()) * (remaining_distance / segment_distance);
                std::cout << "Final point after geodesic addition: " << final_point << std::endl;

                // 시각화
                visualizeDirectionArrow(Point3toEigenVector(current_point), direction, marker_array, marker_id++);
                return Eigen::Vector3d(final_point.x(), final_point.y(), final_point.z());
            }

            distance_traveled += segment_distance;
            current_point = path_point;
            current_face = next_face;
            current_location = next_location;

            Kernel::Vector_3 new_normal_cgal = CGAL::Polygon_mesh_processing::compute_face_normal(next_face, mesh);
            Eigen::Vector3d new_normal = Vector3toEigenVector(new_normal_cgal);

            direction = rotateVectorToNewNormal(direction, old_normal, new_normal);

            // 시각화
            visualizeDirectionArrow(Point3toEigenVector(current_point), direction, marker_array, marker_id++);

            old_normal = new_normal;
        }
        else
        {
            break;
        }
    }

    std::cout << "End point after geodesic addition: " << current_point << std::endl;
    return Eigen::Vector3d(current_point.x(), current_point.y(), current_point.z());
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
double computeGeodesicDistance(const vertex_descriptor &p0, const vertex_descriptor &p1)
{
    Surface_mesh_shortest_path shortest_paths(tmesh);

    // p1을 source point로 추가
    shortest_paths.add_source_point(p1);

    // p0와 p1 사이의 지오데식 거리 계산
    std::pair<double, Surface_mesh_shortest_path::Source_point_iterator> result = shortest_paths.shortest_distance_to_source_points(p0);

    // 거리 반환 (result.first는 지오데식 거리)
    return result.first;
}
// Geodesic interpolation parameters를 계산하는 함수
void updateInterpolationParameters(std::vector<double> &u_values, const vertex_descriptor &p0, const vertex_descriptor &p1, bool chord_length = true)
{
    // p(i-1)과 p(i) 사이의 geodesic distance 계산
    double geodesic_distance = computeGeodesicDistance(p0, p1);

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
    visualizePoints(path_points, marker_array);

    // p1에서 경로 상의 첫 번째 점까지의 벡터 계산 및 정규화 (unit vector)
    Kernel::Vector_3 direction_vector = path_points[1] - path_points[0];
    Eigen::Vector3d unit_vector(direction_vector.x(), direction_vector.y(), direction_vector.z());
    unit_vector.normalize(); // Unit vector로 정규화

    // Geodesic distance 계산
    double geodesic_distance = computeGeodesicDistance(get_closest_vertex_on_face(face1, point1, mesh),
                                                       get_closest_vertex_on_face(face2, point2, mesh));

    // Unit vector에 geodesic distance를 곱하여 최종 결과 계산
    return unit_vector * geodesic_distance;
}
// Geodesic tangent vectors를 계산하는 함수
std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(
    const std::vector<vertex_descriptor> &vertices,
    const std::vector<double> &u_values,
    const Triangle_mesh &mesh, visualization_msgs::MarkerArray marker_array)
{
    std::vector<Eigen::Vector3d> tangent_vectors;

    // 첫 번째 점과 마지막 점의 좌표를 가져옴
    std::cout << "Vertices size: " << vertices.size() << std::endl;
    std::cout << "u_values size: " << u_values.size() << std::endl;

    if (vertices.empty() || u_values.empty())
    {
        throw std::runtime_error("Vertices or u_values are empty!");
    }

    Eigen::Vector3d p_first(mesh.point(vertices.front()).x(), mesh.point(vertices.front()).y(), mesh.point(vertices.front()).z());
    Eigen::Vector3d p_last(mesh.point(vertices.back()).x(), mesh.point(vertices.back()).y(), mesh.point(vertices.back()).z());

    std::cout << "First point: " << p_first.transpose() << std::endl;
    std::cout << "Last point: " << p_last.transpose() << std::endl;

    // 첫 번째 접선 벡터 정의
    if (p_first == p_last)
    {
        std::cout << "First and last points are equal, calculating tangent vector for the first point." << std::endl;

        Eigen::Vector3d p1(mesh.point(vertices[1]).x(), mesh.point(vertices[1]).y(), mesh.point(vertices[1]).z());
        Eigen::Vector3d p_last_prev(mesh.point(vertices[vertices.size() - 2]).x(), mesh.point(vertices[vertices.size() - 2]).y(), mesh.point(vertices[vertices.size() - 2]).z());

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
    for (size_t i = 1; i < vertices.size() - 1; ++i)
    {
        Eigen::Vector3d p_prev(mesh.point(vertices[i - 1]).x(), mesh.point(vertices[i - 1]).y(), mesh.point(vertices[i - 1]).z());
        Eigen::Vector3d p_next(mesh.point(vertices[i + 1]).x(), mesh.point(vertices[i + 1]).y(), mesh.point(vertices[i + 1]).z());

        Eigen::Vector3d tangent_vector = 0.5 * (geodesicSubtract(p_prev, p_next, mesh, marker_array)) / (u_values[i + 1] - u_values[i - 1]);
        tangent_vectors.push_back(tangent_vector);

        std::cout << "Tangent vector " << i << ": " << tangent_vector.transpose() << std::endl;
    }

    // 마지막 접선 벡터 정의
    if (p_first == p_last)
    {
        std::cout << "Calculating tangent vector for the last point." << std::endl;

        Eigen::Vector3d p1(mesh.point(vertices[1]).x(), mesh.point(vertices[1]).y(), mesh.point(vertices[1]).z());
        Eigen::Vector3d p_last_prev(mesh.point(vertices[vertices.size() - 2]).x(), mesh.point(vertices[vertices.size() - 2]).y(), mesh.point(vertices[vertices.size() - 2]).z());

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

// Bézier 곡선의 제어점을 계산하는 함수
std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(
    const std::vector<vertex_descriptor> &vertices,
    const std::vector<double> &u_values,
    const std::vector<Eigen::Vector3d> &tangent_vectors,
    const Triangle_mesh &mesh, visualization_msgs::MarkerArray &marker_array)
{
    std::vector<std::vector<Eigen::Vector3d>> bezier_control_points;
    int marker_id = 0; // marker_id 초기화
    for (size_t i = 0; i < vertices.size() - 1; ++i)
    {
        Eigen::Vector3d p_i(mesh.point(vertices[i]).x(), mesh.point(vertices[i]).y(), mesh.point(vertices[i]).z());
        Eigen::Vector3d p_i1(mesh.point(vertices[i + 1]).x(), mesh.point(vertices[i + 1]).y(), mesh.point(vertices[i + 1]).z());

        const Eigen::Vector3d &tangent_vector_i = tangent_vectors[i];
        const Eigen::Vector3d &tangent_vector_i1 = tangent_vectors[i + 1];

        double distance = (u_values[i + 1] - u_values[i]) / 3.0 * tangent_vector_i.norm();
        Eigen::Vector3d b0 = p_i;
        // Eigen::Vector3d b1 = geodesicAddVector(mesh, Kernel::Point_3(p_i.x(), p_i.y(), p_i.z()), tangent_vector_i.normalized(), distance);
        Eigen::Vector3d b1 = geodesicAddVector(mesh, Kernel::Point_3(p_i.x(), p_i.y(), p_i.z()), tangent_vector_i.normalized(), distance, marker_array, marker_id);
        marker_id++; // b1의 marker_id를 사용한 후 증가
        double distance2 = (u_values[i + 1] - u_values[i]) / 3.0 * tangent_vector_i1.norm();
        // Eigen::Vector3d b2 = geodesicAddVector(mesh, Kernel::Point_3(p_i1.x(), p_i1.y(), p_i1.z()), -tangent_vector_i1.normalized(), distance2);
        Eigen::Vector3d b2 = geodesicAddVector(mesh, Kernel::Point_3(p_i1.x(), p_i1.y(), p_i1.z()), -tangent_vector_i1.normalized(), distance2, marker_array, marker_id);
        marker_id++; // b2의 marker_id를 사용한 후 증가
        Eigen::Vector3d b3 = p_i1;
        bezier_control_points.push_back({p_i, b1, b2, p_i1});

        std::cout << "Computed Bezier control points for segment " << i << ":\n"
                  << "B0: " << b0.transpose() << "\n"
                  << "B1: " << b1.transpose() << "\n"
                  << "B2: " << b2.transpose() << "\n"
                  << "B3: " << b3.transpose() << std::endl;
    }

    return bezier_control_points;
}

// Bézier 곡선을 Bernstein 형식으로 계산하는 함수
std::vector<Eigen::Vector3d> computeBezierCurvePoints(
    const std::vector<Eigen::Vector3d> &control_points,
    int steps)
{
    std::vector<Eigen::Vector3d> curve_points;
    curve_points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i)
    {
        double t = static_cast<double>(i) / steps;
        double u = 1 - t;

        // Bernstein 다항식을 사용하여 Bézier 곡선의 한 점 계산
        Eigen::Vector3d point =
            std::pow(u, 3) * control_points[0] +
            3 * std::pow(u, 2) * t * control_points[1] +
            3 * u * std::pow(t, 2) * control_points[2] +
            std::pow(t, 3) * control_points[3];

        curve_points.push_back(point);
    }

    return curve_points;
}

// Hermite 스플라인을 계산하는 함수 (여러 Bézier 곡선을 연결)
void generate_Hermite_Spline_path(
    const std::vector<std::vector<Eigen::Vector3d>> &bezier_control_points,
    int steps_per_segment)
{
    std::vector<Eigen::Vector3d> hermite_spline;

    for (const auto &control_points : bezier_control_points)
    {
        // 각 Bézier 곡선을 Bernstein 형식으로 계산
        std::vector<Eigen::Vector3d> curve_points = computeBezierCurvePoints(control_points, steps_per_segment);
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
    clicked_points.push_back(msg->point);
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

    std::vector<vertex_descriptor> vertices;      // 선택한 점들을 최근접 vertex로 가정
    std::vector<double> u_values = {0.0};         // u_0 = 0으로 초기화
    std::vector<Eigen::Vector3d> tangent_vectors; // 선택한 점들의 tangent vectors
    int steps_per_segment = 20;                   // 각 Bézier 곡선을 몇 단계로 세분화할지 결정
                                                  // 모든 마커들을 담을 MarkerArray 생성
    visualization_msgs::MarkerArray marker_array;
    ros::Rate r(30);

    while (ros::ok())
    {
        ros::spinOnce();
        if (new_waypoints)
        {
            // 클릭한 점의 좌표를 이용해 가장 가까운 vertex_descriptor를 찾음
            vertex_descriptor new_vertex = findClosestVertex(clicked_points.back());
            vertices.push_back(new_vertex);

            // u_values 업데이트
            if (vertices.size() > 1)
            {
                updateInterpolationParameters(u_values, vertices[vertices.size() - 2], new_vertex, true);
            }
            if (vertices.size() > 1 && u_values.size() > 1)
            {
                // Geodesic tangent vectors 계산
                tangent_vectors = calculateGeodesicTangentVectors(vertices, u_values, tmesh, marker_array);

                std::vector<Eigen::Vector3d> points; // Tangent vectors의 시작 위치 (vertices에 해당하는 실제 좌표)
                for (const auto &vertex : vertices)
                {
                    const auto &point = tmesh.point(vertex);
                    std::cout << "Vertex: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
                    points.push_back(Eigen::Vector3d(point.x(), point.y(), point.z()));
                } // Tangent vectors 시각화
                visualizeTangentVectors(tangent_vectors, points, marker_array);

                // 터미널에 출력
                std::cout << "Vertices:" << std::endl;
                for (const auto &vertex : vertices)
                {
                    const auto &point = tmesh.point(vertex);
                    std::cout << "Vertex: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
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
                std::vector<std::vector<Eigen::Vector3d>> bezier_control_points = computeBezierControlPoints(vertices, u_values, tangent_vectors, tmesh, marker_array);

                // Hermite 스플라인을 Bernstein 형식으로 계산

                generate_Hermite_Spline_path(bezier_control_points, steps_per_segment);
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
