#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nrs_vision_rviz/Waypoint.h>
#include <nrs_vision_rviz/Waypoints.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Ray_3.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// === CGAL 타입 정의 ===
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;
typedef Traits::Point_3 Point_3;
typedef Kernel::Ray_3 Ray_3;
typedef Kernel::Vector_3 Vector_3;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Triangle_mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> AABB_tree;

std::vector<geometry_msgs::Point> original_points;

std::vector<geometry_msgs::Point> clicked_points;
std::vector<Eigen::Vector3d> selected_points; // Projected [clicked_points] onto Mesh surface
Triangle_mesh mesh;
AABB_tree tree;
bool clickedPointReceived = false;

bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &mesh)
{
    if (mesh.faces().empty())
    {
        std::cerr << "Mesh is empty, cannot build AABB tree." << std::endl;
        return false;
    }

    AABB_tree tree(mesh.faces().begin(), mesh.faces().end(), mesh);
    tree.build();

    auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, mesh);
    face = result.first;
    location = result.second;

    if (face == Triangle_mesh::null_face())
    {
        std::cerr << "Failed to locate face for point: " << point << std::endl;
        return false;
    }

    return true;
}

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    std::cout << "-----------------------------------new waypoints comming---------------------------------" << std::endl;

    Kernel::Point_3 clicked_point(msg->point.x, msg->point.y, msg->point.z);

    face_descriptor face;
    Surface_mesh_shortest_path::Barycentric_coordinates location;

    if (!locate_face_and_point(clicked_point, face, location, mesh))
    {
        std::cerr << "Failed to locate the clicked point on the mesh." << std::endl;
        return;
    }

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

    Kernel::Point_3 projected_point = CGAL::barycenter(v1, location[0], v2, location[1], v3, location[2]);

    Eigen::Vector3d projected_point_eigen(projected_point.x(), projected_point.y(), projected_point.z());

    clicked_points.push_back(msg->point);
    selected_points.push_back(projected_point_eigen);
    clickedPointReceived = true;
    ROS_INFO("Clicked point received: (%f, %f, %f)", msg->point.x, msg->point.y, msg->point.z);
}

// === AABB Tree 생성 및 메쉬의 최대 z 값 찾는 함수 ===
double initializeMeshAndGetMaxZ(const std::string &mesh_file_path, Triangle_mesh &mesh, AABB_tree &tree)
{
    // STL 파일 로드
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input || !CGAL::IO::read_STL(input, mesh))
    {
        ROS_ERROR("Failed to load mesh file.");
        exit(-1);
    }

    // AABB Tree 생성
    tree = AABB_tree(faces(mesh).begin(), faces(mesh).end(), mesh);
    tree.accelerate_distance_queries();

    // 메쉬에서 가장 높은 z 값 찾기
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto &v : vertices(mesh))
    {
        const Point_3 &p = mesh.point(v);
        if (p.z() > max_z)
        {
            max_z = p.z();
        }
    }

    return max_z + 0.05; // 가장 높은 z 값보다 5cm 위로 설정
}

// === 나선형 경로 생성 함수 ===
std::vector<Point_3> generateSpiralPath(double x_origin, double y_origin, double projection_z, double r_start, double r_end, int num_points = 100)
{
    std::vector<Point_3> path_2D;
    double theta_max = 10 * M_PI;                     // 5바퀴 (10π rad)
    double k = std::log(r_end / r_start) / theta_max; // 감쇠 계수

    for (int i = 0; i < num_points; ++i)
    {
        double theta = i * (theta_max / num_points);
        double r = r_start * std::exp(k * theta);
        double x = x_origin + r * std::cos(theta);
        double y = y_origin + r * std::sin(theta);
        path_2D.push_back(Point_3(x, y, projection_z));
    }

    return path_2D;
}

// === 2D 경로를 3D 메쉬에 수직 투영하는 함수 ===
std::vector<Point_3> projectPathOntoMesh(const std::vector<Point_3> &path_2D, AABB_tree &tree)
{
    std::vector<Point_3> projected_points;

    for (const auto &p : path_2D)
    {
        Ray_3 vertical_ray(p, Vector_3(0, 0, -1)); // z- 방향으로 광선 생성

        // AABB Tree를 사용하여 가장 가까운 교차점 찾기
        auto intersection = tree.first_intersection(vertical_ray);

        if (intersection)
        {
            if (const Point_3 *proj_point = boost::get<Point_3>(&intersection->first))
            {
                projected_points.push_back(*proj_point);
                // std::cout << "Projected Point: " << *proj_point << std::endl;
            }
        }
        else
        {
            std::cerr << "No intersection found for point: " << p << std::endl;
        }
    }

    return projected_points;
}

Eigen::Vector3d getFaceNormal(const geometry_msgs::Point &ros_point, const Triangle_mesh &mesh)
{
    // ROS geometry_msgs::Point -> CGAL Point_3 변환
    Point_3 cgal_point(ros_point.x, ros_point.y, ros_point.z);

    // locate_face_and_point()를 사용하여 서피스에서 점과 면을 찾음
    face_descriptor face;
    CGAL::cpp11::array<double, 3> location; // Barycentric coordinates

    if (!locate_face_and_point(cgal_point, face, location, mesh))
    {
        ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", ros_point.x, ros_point.y, ros_point.z);
        return Eigen::Vector3d::Zero(); // 실패 시 0 벡터 반환
    }

    // 면의 노멀 벡터 계산
    Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);

    // CGAL 노멀 벡터를 Eigen::Vector3d로 변환
    Eigen::Vector3d eigen_normal(normal.x(), normal.y(), normal.z());

    // 노멀 벡터 정규화
    eigen_normal.normalize();

    return eigen_normal; // 노멀 벡터 반환
}

std::vector<geometry_msgs::Point> generate_segment(std::vector<geometry_msgs::Point> &original_points, int option, const Triangle_mesh &mesh)
{
    // 첫 번째 점과 마지막 점에서 face normal vector 구하기
    geometry_msgs::Point start_point = original_points.front();
    geometry_msgs::Point end_point = original_points.back();

    // 첫 번째와 마지막 점에서 face의 normal vector 구하기
    Eigen::Vector3d start_normal = getFaceNormal(start_point, mesh);
    Eigen::Vector3d end_normal = getFaceNormal(end_point, mesh);

    geometry_msgs::Point start_approach;
    geometry_msgs::Point end_retreat;
    geometry_msgs::Point home_position;

    if (option == 1) // approach
    {
        start_approach.x = start_point.x + 0.1 * start_normal.x();
        start_approach.y = start_point.y + 0.1 * start_normal.y();
        start_approach.z = start_point.z + 0.1 * start_normal.z();
        std::vector<geometry_msgs::Point> first_segment{start_approach, original_points.front()};

        return first_segment;
    }
    else if (option == 2) // retreat
    {
        end_retreat.x = end_point.x + 0.1 * end_normal.x();
        end_retreat.y = end_point.y + 0.1 * end_normal.y();
        end_retreat.z = end_point.z + 0.1 * end_normal.z();
        std::vector<geometry_msgs::Point> last_segment{original_points.back(), end_retreat};

        return last_segment;
    }
    else if (option == 3) // home
    {

        end_retreat.x = end_point.x + 0.1 * end_normal.x();
        end_retreat.y = end_point.y + 0.1 * end_normal.y();
        end_retreat.z = end_point.z + 0.1 * end_normal.z();

        home_position.x = 0.573;
        home_position.y = -0.127;
        home_position.z = 0.25;
        std::vector<geometry_msgs::Point> home_segment{end_retreat, home_position};

        return home_segment;
    }
}

std::vector<geometry_msgs::Point> interpolatePoints(
    const std::vector<geometry_msgs::Point> &points,
    double desired_interval,
    int option)
{
    std::vector<geometry_msgs::Point> interpolated_points;

    // 1. 원래 웨이포인트 간의 누적 지오데식 거리 계산
    std::vector<double> cumulative_distances(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i)
    {
        Eigen::Vector3d p0(points[i - 1].x, points[i - 1].y, points[i - 1].z);
        Eigen::Vector3d p1(points[i].x, points[i].y, points[i].z);
        cumulative_distances[i] = cumulative_distances[i - 1] + (p1 - p0).norm();
        // cumulative_distances[i] = cumulative_distances[i - 1] + computeGeodesicDistance(p0, p1, mesh);
    }

    if (option == 1)
    {
        // Option 1: 일정한 간격으로 보간
        double current_distance = 0.0;
        size_t j = 1;
        while (j < points.size())
        {
            if (cumulative_distances[j] >= current_distance + desired_interval)
            {
                double t = (current_distance + desired_interval - cumulative_distances[j - 1]) / (cumulative_distances[j] - cumulative_distances[j - 1]);

                geometry_msgs::Point interpolated_point;
                interpolated_point.x = points[j - 1].x + t * (points[j].x - points[j - 1].x);
                interpolated_point.y = points[j - 1].y + t * (points[j].y - points[j - 1].y);
                interpolated_point.z = points[j - 1].z + t * (points[j].z - points[j - 1].z);

                interpolated_points.push_back(interpolated_point);
                current_distance += desired_interval;
            }
            else
            {
                ++j;
            }
        }
    }
    else if (option == 2)
    {
        // Option 2: 가변 간격 사용
        double total_distance = cumulative_distances.back();
        double transition_length = 0.03; // 3cm

        auto computeVariableInterval = [&](double current_distance) -> double
        {
            if (current_distance < transition_length) // 초반 3cm 구간
            {
                double scale = 0.5 * (1 - cos((current_distance / transition_length) * M_PI));       // 0에서 π까지의 사인 변화
                return desired_interval / 6.0 + scale * (desired_interval - desired_interval / 6.0); // 점진적 변화
            }
            else if (current_distance > total_distance - transition_length) // 끝부분 3cm 구간
            {
                double remaining_distance = total_distance - current_distance;
                double scale = 0.5 * (1 - cos((remaining_distance / transition_length) * M_PI));     // 끝에서 다시 느려짐
                return desired_interval / 6.0 + scale * (desired_interval - desired_interval / 6.0); // 점진적 변화
            }
            else // 중간 구간은 일정한 간격 유지
            {
                return desired_interval;
            }
        };

        double current_distance = 0.0;
        size_t j = 1;
        while (j < points.size())
        {
            double current_interval = computeVariableInterval(current_distance); // 가변적 간격 계산

            if (cumulative_distances[j] >= current_distance + current_interval)
            {
                double t = (current_distance + current_interval - cumulative_distances[j - 1]) / (cumulative_distances[j] - cumulative_distances[j - 1]);

                geometry_msgs::Point interpolated_point;
                interpolated_point.x = points[j - 1].x + t * (points[j].x - points[j - 1].x);
                interpolated_point.y = points[j - 1].y + t * (points[j].y - points[j - 1].y);
                interpolated_point.z = points[j - 1].z + t * (points[j].z - points[j - 1].z);

                interpolated_points.push_back(interpolated_point);
                current_distance += current_interval; // 가변적 간격으로 증가
            }
            else
            {
                ++j;
            }
        }
    }

    return interpolated_points;
}

// 서피스에서 노멀 벡터를 계산하고 웨이포인트로 변환하는 함수
nrs_vision_rviz::Waypoints convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const std::vector<geometry_msgs::Point> &reference_points, const Triangle_mesh &mesh, int option)
{
    nrs_vision_rviz::Waypoints waypoints;

    Point_3 start_point(reference_points.begin()->x, reference_points.begin()->y, reference_points.begin()->z);
    Point_3 end_point(reference_points[reference_points.size() - 2].x, reference_points[reference_points.size() - 2].y, reference_points[reference_points.size() - 2].z);
    face_descriptor start_face, end_face;
    CGAL::cpp11::array<double, 3> start_location, end_location;
    // 서피스에서 점과 면을 찾음
    locate_face_and_point(start_point, start_face, start_location, mesh);
    // 서피스에서 점과 면을 찾음
    locate_face_and_point(end_point, end_face, end_location, mesh);
    // 노멀 벡터 계산
    Kernel::Vector_3 start_normal = CGAL::Polygon_mesh_processing::compute_face_normal(start_face, mesh);
    // 노멀 벡터 계산
    Kernel::Vector_3 end_normal = CGAL::Polygon_mesh_processing::compute_face_normal(end_face, mesh);

    if (option == 1) // approach_segment
    {
        tf2::Vector3 z_axis(-start_normal.x(), -start_normal.y(), -start_normal.z());
        z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();
        // 회전 행렬 계산
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);

        for (auto &point : points)
        {
            nrs_vision_rviz::Waypoint waypoint_msg;
            waypoint_msg.pose.position.x = point.x;
            waypoint_msg.pose.position.y = point.y;
            waypoint_msg.pose.position.z = point.z;

            waypoint_msg.pose.orientation.w = orientation.w();
            waypoint_msg.pose.orientation.x = orientation.x();
            waypoint_msg.pose.orientation.y = orientation.y();
            waypoint_msg.pose.orientation.z = orientation.z();

            waypoints.waypoints.push_back(waypoint_msg);
        }
    }
    else if (option == 2) // original_segment
    {

        for (const auto &point : points)
        {
            Kernel::Point_3 cgal_point(point.x, point.y, point.z);
            face_descriptor face;
            CGAL::cpp11::array<double, 3> location;

            // 각 점이 속한 face와 바리센트릭 좌표를 찾아냅니다.
            if (!locate_face_and_point(cgal_point, face, location, mesh))
            {
                ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", point.x, point.y, point.z);
                continue;
            }

            // 삼각형 면의 세 정점의 vertex descriptor를 가져옵니다.
            Triangle_mesh::Halfedge_index h = mesh.halfedge(face);
            vertex_descriptor v0 = mesh.source(h);
            vertex_descriptor v1 = mesh.target(h);
            vertex_descriptor v2 = mesh.target(mesh.next(h));

            // 각 정점의 노멀 벡터를 계산합니다.
            Kernel::Vector_3 normal_v0 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v0, mesh);
            Kernel::Vector_3 normal_v1 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v1, mesh);
            Kernel::Vector_3 normal_v2 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v2, mesh);

            // 바리센트릭 좌표를 사용하여 보간된 점의 노멀 벡터를 계산합니다.
            Kernel::Vector_3 interpolated_normal = location[0] * normal_v0 +
                                                   location[1] * normal_v1 +
                                                   location[2] * normal_v2;

            tf2::Vector3 z_axis(-interpolated_normal.x(), -interpolated_normal.y(), -interpolated_normal.z());
            z_axis.normalize();
            tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
            tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
            x_axis = y_axis.cross(z_axis).normalized();
            // 회전 행렬 계산
            tf2::Matrix3x3 orientation_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z());

            tf2::Quaternion orientation;
            orientation_matrix.getRotation(orientation);

            // 결과를 웨이포인트 형식으로 저장합니다.
            nrs_vision_rviz::Waypoint waypoint_msg;
            waypoint_msg.pose.position.x = point.x;
            waypoint_msg.pose.position.y = point.y;
            waypoint_msg.pose.position.z = point.z;

            waypoint_msg.pose.orientation.w = orientation.w();
            waypoint_msg.pose.orientation.x = orientation.x();
            waypoint_msg.pose.orientation.y = orientation.y();
            waypoint_msg.pose.orientation.z = orientation.z();

            waypoints.waypoints.push_back(waypoint_msg);
        }
    }

    else if (option == 3)
    { // retreat segment
        tf2::Vector3 z_axis(-end_normal.x(), -end_normal.y(), -end_normal.z());
        z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();
        // 회전 행렬 계산
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);

        for (auto &point : points)
        {
            nrs_vision_rviz::Waypoint waypoint_msg;
            waypoint_msg.pose.position.x = point.x;
            waypoint_msg.pose.position.y = point.y;
            waypoint_msg.pose.position.z = point.z;

            waypoint_msg.pose.orientation.w = orientation.w();
            waypoint_msg.pose.orientation.x = orientation.x();
            waypoint_msg.pose.orientation.y = orientation.y();
            waypoint_msg.pose.orientation.z = orientation.z();

            waypoints.waypoints.push_back(waypoint_msg);
        }
    }

    else if (option == 4) // home segment
    {
        tf2::Vector3 z_axis(-end_normal.x(), -end_normal.y(), -end_normal.z());
        z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();
        // 회전 행렬 계산
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion orientation;
        orientation_matrix.getRotation(orientation);
        for (auto &point : points)
        {
            nrs_vision_rviz::Waypoint waypoint_msg;
            waypoint_msg.pose.position.x = point.x;
            waypoint_msg.pose.position.y = point.y;
            waypoint_msg.pose.position.z = point.z;

            waypoint_msg.pose.orientation.w = orientation.w();
            waypoint_msg.pose.orientation.x = orientation.x();
            waypoint_msg.pose.orientation.y = orientation.y();
            waypoint_msg.pose.orientation.z = orientation.z();

            waypoints.waypoints.push_back(waypoint_msg);
        }
    }

    else if (option == 5)
    {
        for (const auto &point : points)
        {
            Kernel::Point_3 cgal_point(point.x, point.y, point.z);
            face_descriptor face;
            CGAL::cpp11::array<double, 3> location;
            // 각 점이 속한 face를 찾아냅니다.
            if (!locate_face_and_point(cgal_point, face, location, mesh))
            {
                ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", point.x, point.y, point.z);
                continue;
            }

            // face의 법선 벡터를 계산합니다.
            Kernel::Vector_3 face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);

            // 법선 벡터를 기준으로 z-axis를 정의합니다.
            tf2::Vector3 z_axis(-face_normal.x(), -face_normal.y(), -face_normal.z());
            z_axis.normalize();

            // 임의의 x-axis를 설정하고 z-axis에 직교하는 y-axis를 계산합니다.
            tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
            tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
            x_axis = y_axis.cross(z_axis).normalized();

            // 회전 행렬을 계산합니다.
            tf2::Matrix3x3 orientation_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z());

            tf2::Quaternion orientation;
            orientation_matrix.getRotation(orientation);

            // 결과를 웨이포인트 형식으로 저장합니다.
            nrs_vision_rviz::Waypoint waypoint_msg;
            waypoint_msg.pose.position.x = point.x;
            waypoint_msg.pose.position.y = point.y;
            waypoint_msg.pose.position.z = point.z;

            waypoint_msg.pose.orientation.w = orientation.w();
            waypoint_msg.pose.orientation.x = orientation.x();
            waypoint_msg.pose.orientation.y = orientation.y();
            waypoint_msg.pose.orientation.z = orientation.z();

            waypoints.waypoints.push_back(waypoint_msg);
        }
    }

    return waypoints;
}

void clearFile(const std::string &file_path)
{
    // 파일을 "truncate" 모드로 열어서 내용 제거
    std::ofstream file(file_path, std::ofstream::trunc);
    if (file.is_open())
    {

        file.close();
    }
    else
    {
        std::cerr << "Failed to open file: " << file_path << std::endl;
    }
}

void saveWayPointsTOFile(const nrs_vision_rviz::Waypoints &waypoints, const std::string &filename, double force)
{
    // 파일 열기
    std::ofstream outfile(filename, std::ios_base::app);

    if (!outfile.is_open())
    {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }
    outfile << std::fixed << std::setprecision(6);

    // 각 웨이포인트의 데이터를 파일로 저장
    for (const auto &waypoint : waypoints.waypoints)
    {
        // 1. Position (x, y, z)
        double x = waypoint.pose.position.x;
        double y = waypoint.pose.position.y;
        double z = waypoint.pose.position.z;

        // 2. Orientation (Quaternion -> RPY 변환)
        tf2::Quaternion q(
            waypoint.pose.orientation.x,
            waypoint.pose.orientation.y,
            waypoint.pose.orientation.z,
            waypoint.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 3. 파일에 기록
        outfile << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << 0.0 << " " << 0.0 << " " << force << "\n";
    }
    // 파일 닫기
    outfile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_projection_node");
    ros::NodeHandle nh;
    ros::Duration(5.0).sleep();

    // 웨이포인트 간의 원하는 간격
    double desired_interval = 0.00006;
    // rviz에서 선택한 점의 위치 정보를 받아노는 Subscriber
    ros::Subscriber point_sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    // 퍼블리셔 선언
    ros::Publisher interpolated_waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("interpolated_waypoints_with_normals", 10);
    // 파일 퍼블리셔 선언
    ros::Publisher file_pub = nh.advertise<std_msgs::String>("path_publisher", 10);
    // STL 파일 경로
    std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/fenda_wrap.stl";

    // 메쉬 초기화 및 최대 z 값 계산
    double projection_z = initializeMeshAndGetMaxZ(mesh_file_path, mesh, tree);

    // 클릭된 포인트가 수신될 때까지 대기
    ROS_INFO("Waiting for clicked point...");
    while (ros::ok() && !clickedPointReceived)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Clicked point received. Proceeding to mesh initialization.");

    // 나선형 경로 생성
    std::vector<Point_3> path_2D = generateSpiralPath(selected_points[0].x(), selected_points[0].y(), projection_z, 0.06, 0.01, 100);

    // 2D 경로를 3D 메쉬에 투영
    std::vector<Point_3> projected_points = projectPathOntoMesh(path_2D, tree);

    auto start_time = std::chrono::high_resolution_clock::now();
    for (const auto &p : projected_points)
    {
        geometry_msgs::Point ros_point;
        ros_point.x = p.x();
        ros_point.y = p.y();
        ros_point.z = p.z();
        original_points.push_back(ros_point);
    }
    // 접근 segment 생성
    std::vector<geometry_msgs::Point> approach_segment = generate_segment(original_points, 1, mesh);

    // 후퇴 segment 생성
    std::vector<geometry_msgs::Point> retreat_segment = generate_segment(original_points, 2, mesh);

    // 복귀 segment 생성
    std::vector<geometry_msgs::Point> home_segment = generate_segment(original_points, 3, mesh);

    //------------------------------------------------------------------------------------------------------------------

    // 시각화용 접근 구간 interpolation
    std::vector<geometry_msgs::Point> visual_approach_interpolated = interpolatePoints(approach_segment, 0.001, 1);

    // 시각화용 폴리싱 구간 interpolation (원래 waypoints)
    std::vector<geometry_msgs::Point> visual_original_interpolated = interpolatePoints(original_points, 0.001, 1);

    // 시각화용 후퇴 구간 interpolation
    std::vector<geometry_msgs::Point> visual_retreat_interpolated = interpolatePoints(retreat_segment, 0.001, 1);
    std::cout << "interpolation done" << std::endl;
    // 접근 구간의 방향 벡터 설정
    nrs_vision_rviz::Waypoints visual_approach_waypoints = convertToWaypoints(visual_approach_interpolated, original_points, mesh, 1);

    // 폴리싱 구간의 방향 벡터 설정
    nrs_vision_rviz::Waypoints visual_original_waypoints = convertToWaypoints(visual_original_interpolated, original_points, mesh, 2);

    // 후퇴 구간의 방향 벡터 설정
    nrs_vision_rviz::Waypoints visual_retreat_waypoints = convertToWaypoints(visual_retreat_interpolated, original_points, mesh, 3);
    std::cout << "converttowaypoints done" << std::endl;
    nrs_vision_rviz::Waypoints visual_final_waypoints;
    // 전체 구간 waypoints 합치기
    visual_final_waypoints.waypoints.insert(visual_final_waypoints.waypoints.end(), visual_approach_waypoints.waypoints.begin(), visual_approach_waypoints.waypoints.end());
    visual_final_waypoints.waypoints.insert(visual_final_waypoints.waypoints.end(), visual_original_waypoints.waypoints.begin(), visual_original_waypoints.waypoints.end());
    visual_final_waypoints.waypoints.insert(visual_final_waypoints.waypoints.end(), visual_retreat_waypoints.waypoints.begin(), visual_retreat_waypoints.waypoints.end());
    //rviz 시각화를 위해 publish
    std::cout << "waypoints adding done" << std::endl;
    nrs_vision_rviz::Waypoints visual_waypoints_msg;
    visual_waypoints_msg = visual_final_waypoints;
    interpolated_waypoints_pub.publish(visual_waypoints_msg);
    std::cout << "publishing done" << std::endl;
    //-----------------------------------------------------------------------------------------------------------------
    // 제어용 접근 구간 interpolation
    std::vector<geometry_msgs::Point> control_approach_interpolated = interpolatePoints(approach_segment, desired_interval, 2);

    // 제어용 폴리싱 구간 interpolation (원래 waypoints)
    std::vector<geometry_msgs::Point> control_original_interpolated = interpolatePoints(original_points, desired_interval, 2);

    // 제어용 후퇴 구간 interpolation
    std::vector<geometry_msgs::Point> control_retreat_interpolated = interpolatePoints(retreat_segment, desired_interval, 2);

    // 제어용 홈 포지션 복귀 구간 interpolation
    std::vector<geometry_msgs::Point> control_home_interpolated = interpolatePoints(home_segment, desired_interval, 2);

    // 첫 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_approach_waypoints = convertToWaypoints(control_approach_interpolated, control_original_interpolated, mesh, 1);
    // 두 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_original_waypoints = convertToWaypoints(control_original_interpolated, original_points, mesh, 2);
    // 세 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_retreat_waypoints = convertToWaypoints(control_retreat_interpolated, control_original_interpolated, mesh, 3);
    // 네 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_home_waypoints = convertToWaypoints(control_home_interpolated, control_original_interpolated, mesh, 4);

    // 파일 경로
    std::string file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/projected_final_waypoints.txt";

    clearFile(file_path);

    saveWayPointsTOFile(control_approach_waypoints, file_path, 0.0);
    saveWayPointsTOFile(control_original_waypoints, file_path, 10.0);
    saveWayPointsTOFile(control_retreat_waypoints, file_path, 0.0);
    saveWayPointsTOFile(control_home_waypoints, file_path, 0.0);

    ROS_INFO("--------------------------------------\n Saved %lu final waypoints \n --------------------------------------",
             control_approach_waypoints.waypoints.size() + control_original_waypoints.waypoints.size() + control_retreat_waypoints.waypoints.size() + control_home_waypoints.waypoints.size());

    // 소요 시간 측정 및 출력
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    std::cout << "interpolation & Normal smoothing time: " << duration << " s" << std::endl;

    return 0;
}
