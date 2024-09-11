#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nrs_path_planning/Waypoint.h>
#include <nrs_path_planning/Waypoints.h>

// CGAL 관련 헤더
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL_reader.h>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <fstream>
#include <cmath>
#include <map>
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

Triangle_mesh mesh;
Tree *tree;
Surface_mesh_shortest_path *shortest_paths;

// STL 파일을 읽어 Triangle_mesh 객체로 변환하는 함수
bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh);

// Find closest [face and barycentric coordinate on Mesh] with points
bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &mesh);

// Calculate GeodesicDistance between two points
double computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh);

// 일정한 간격을 가진 웨이포인트 생성 함수
std::vector<geometry_msgs::Point> interpolatePoints(const std::vector<geometry_msgs::Point> &points, double desired_interval, const Triangle_mesh &mesh);

// 점과 그 노멀 벡터를 포함하는 Waypoint 벡터를 생성하는 함수
std::vector<nrs_path_planning::Waypoint> generateWaypointsWithNormals(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh &mesh);

// 서피스에서 노멀 벡터를 계산하고 웨이포인트로 변환하는 함수
std::vector<nrs_path_planning::Waypoint> convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh &mesh);

// Save interpolated waypoints to a text file
void saveWaypointsToFile(const std::vector<nrs_path_planning::Waypoint> &waypoints, const std::string &filename);

// 함수: Waypoints를 입력받아 x, y, z, yaw, roll, pitch 값을 계산하고 텍스트 파일에 저장
<<<<<<< HEAD
void savePosesToFile(const std::vector<nrs_vision_rviz::Waypoint> &waypoints, const std::string &filename, double force);
=======
void savePosesToFile(const std::vector<nrs_path_planning::Waypoint> &waypoints, const std::string &filename);
>>>>>>> 745743660d549b29ec120686157a19684c5f67e3

// 콜백 함수: 웨이포인트 보간, 노멀 계산 및 퍼블리시
void waypointsCallback(const nrs_path_planning::Waypoints::ConstPtr &msg, ros::Publisher &pub, double desired_interval, const Triangle_mesh &mesh);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "waypoints_interpolator");
    ros::NodeHandle nh;

    // 웨이포인트 간의 원하는 간격 (예: 0.05m 간격)
    double desired_interval;
    nh.param("desired_interval", desired_interval, 0.001); // 파라미터 서버에서 간격을 가져오거나 기본값 0.05 사용

    // Triangle_mesh 로드 또는 생성
    Triangle_mesh mesh;
    std::ifstream stl_file("/home/nrs/catkin_ws/src/nrs_path_planning/mesh/lid_wrap.stl"); // STL 파일 경로를 설정
    if (!stl_file || !read_stl_file(stl_file, mesh))
    {
        ROS_ERROR("Failed to load or process STL file.");
        return 1;
    }

    // 퍼블리셔 선언
    ros::Publisher interpolated_waypoints_pub = nh.advertise<nrs_path_planning::Waypoints>("interpolated_waypoints_with_normals", 10);

    // 웨이포인트 구독 및 콜백 함수 설정
    ros::Subscriber waypoints_sub = nh.subscribe<nrs_path_planning::Waypoints>("waypoints_with_normals", 10,
                                                                             boost::bind(waypointsCallback, _1, boost::ref(interpolated_waypoints_pub), desired_interval, boost::cref(mesh)));

    ros::spin();

    return 0;
}

// STL 파일을 읽어 Triangle_mesh 객체로 변환하는 함수
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

// Find closest [face and barycentric coordinate on Mesh] with points
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

    return true;
}

// getFaceNormal 함수
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

// 일정한 간격을 가진 웨이포인트 생성 함수
std::vector<geometry_msgs::Point> interpolatePoints(const std::vector<geometry_msgs::Point> &points, double desired_interval, const Triangle_mesh &mesh)
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

    // 2. 일정한 간격으로 보간된 점들을 추가
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

    return interpolated_points;
}

// 점과 그 노멀 벡터를 포함하는 Waypoint 벡터를 생성하는 함수
std::vector<nrs_path_planning::Waypoint> generateWaypointsWithNormals(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh &mesh)
{
    std::vector<nrs_path_planning::Waypoint> waypoints;

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

        // 결과를 웨이포인트 형식으로 저장합니다.
        nrs_path_planning::Waypoint waypoint_msg;
        waypoint_msg.point.x = point.x;
        waypoint_msg.point.y = point.y;
        waypoint_msg.point.z = point.z;

        waypoint_msg.normal.x = -interpolated_normal.x();
        waypoint_msg.normal.y = -interpolated_normal.y();
        waypoint_msg.normal.z = -interpolated_normal.z();

        waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}

// 서피스에서 노멀 벡터를 계산하고 웨이포인트로 변환하는 함수
std::vector<nrs_path_planning::Waypoint> convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh &mesh)
{
    std::vector<nrs_path_planning::Waypoint> waypoints;

    for (const auto &point : points)
    {
        Point_3 cgal_point(point.x, point.y, point.z);
        face_descriptor face;
        CGAL::cpp11::array<double, 3> location;

        // 서피스에서 점과 면을 찾음
        if (!locate_face_and_point(cgal_point, face, location, mesh))
        {
            ROS_ERROR("Failed to locate face and point for point: [%f, %f, %f]", point.x, point.y, point.z);
            continue;
        }

        // 노멀 벡터 계산
        Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);

        // 웨이포인트 생성
        nrs_path_planning::Waypoint waypoint_msg;
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

// Save interpolated waypoints to a text file
void saveWaypointsToFile(const std::vector<nrs_path_planning::Waypoint> &waypoints, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }

    // Write header
    file << "x y z normal_x normal_y normal_z\n";

    for (const auto &waypoint : waypoints)
    {
        file << waypoint.point.x << " "
             << waypoint.point.y << " "
             << waypoint.point.z << " "
             << waypoint.normal.x << " "
             << waypoint.normal.y << " "
             << waypoint.normal.z << "\n";
    }

    file.close();
    ROS_INFO("Waypoints saved to %s", filename.c_str());
}
// 첫 번째 및 세 번째 구간: 고정된 RPY 값을 사용하여 텍스트 파일에 저장
void saveFixedRPYPoseToFile(const std::vector<nrs_vision_rviz::Waypoint> &waypoints, const std::string &filename, const nrs_vision_rviz::Waypoint &reference_waypoint, double force)
{
    // 첫 번째 웨이포인트를 기준으로 RPY 값을 계산
    tf2::Vector3 z_axis(reference_waypoint.normal.x, reference_waypoint.normal.y, reference_waypoint.normal.z);
    z_axis.normalize();

    tf2::Vector3 x_axis;
    if (waypoints.size() > 1)
    {
        // 첫 번째 웨이포인트와 두 번째 웨이포인트 사이의 방향 벡터 계산
        tf2::Vector3 next_point(waypoints[1].point.x, waypoints[1].point.y, waypoints[1].point.z);
        tf2::Vector3 current_point(waypoints[0].point.x, waypoints[0].point.y, waypoints[0].point.z);
        x_axis = (next_point - current_point).normalized();
    }
    else
    {
        // 점이 하나만 있는 경우 진행 방향은 임의로 설정 (예: X축 방향)
        x_axis.setValue(1.0, 0.0, 0.0);
    }

    // Y축은 Z축과 X축의 외적
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();

    // 회전 행렬 계산
    tf2::Matrix3x3 rotation_matrix(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());

    // roll, pitch, yaw 계산
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    // 파일 열기 (append 모드로 열어서 내용을 추가)
    std::ofstream file(filename, std::ios_base::app); // 'app' 모드로 파일에 내용을 추가
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // 고정된 RPY 값을 모든 웨이포인트에 적용
    for (const auto &waypoint : waypoints)
    {
        // 현재 웨이포인트의 위치 (x, y, z)
        double x = waypoint.point.x;
        double y = waypoint.point.y;
        double z = waypoint.point.z;

        // 고정된 yaw, roll, pitch 값으로 파일에 기록
        file << x << ", " << y << ", " << z << ", " << yaw << ", " << roll << ", " << pitch << ", " << force << "\n";
    }

    // 파일 닫기
    file.close();
    std::cout << "Fixed RPY Poses saved to " << filename << std::endl;
}

// 함수: Waypoints를 입력받아 x, y, z, yaw, roll, pitch 값을 계산하고 텍스트 파일에 저장
<<<<<<< HEAD
void savePosesToFile(const std::vector<nrs_vision_rviz::Waypoint> &waypoints, const std::string &filename, double force)
=======
void savePosesToFile(const std::vector<nrs_path_planning::Waypoint> &waypoints, const std::string &filename)
>>>>>>> 745743660d549b29ec120686157a19684c5f67e3
{
    // 파일 열기 (append 모드로 열어서 내용을 추가)
    std::ofstream file(filename, std::ios_base::app); // 'app' 모드로 파일에 내용을 추가
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // 파일에 헤더 작성
    file << "x, y, z, yaw, roll, pitch, force\n";

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        // 현재 웨이포인트의 위치 (x, y, z)
        double x = waypoints[i].point.x;
        double y = waypoints[i].point.y;
        double z = waypoints[i].point.z;

        // Z축: 웨이포인트의 normal vector (법선 벡터)
        tf2::Vector3 z_axis(waypoints[i].normal.x, waypoints[i].normal.y, waypoints[i].normal.z);
        z_axis.normalize(); // 법선 벡터를 단위 벡터로 정규화

        // X축: 진행 방향 벡터 (현재 웨이포인트와 다음 웨이포인트 사이)
        tf2::Vector3 x_axis;
        if (i < waypoints.size() - 1) // 마지막 웨이포인트가 아닐 때만 계산
        {
            tf2::Vector3 next_point(waypoints[i + 1].point.x, waypoints[i + 1].point.y, waypoints[i + 1].point.z);
            tf2::Vector3 current_point(waypoints[i].point.x, waypoints[i].point.y, waypoints[i].point.z);
            x_axis = (next_point - current_point).normalized(); // 방향 벡터를 단위 벡터로 정규화
        }
        else
        {
            // 마지막 웨이포인트는 이전 웨이포인트와의 차이로 방향을 설정
            tf2::Vector3 prev_point(waypoints[i - 1].point.x, waypoints[i - 1].point.y, waypoints[i - 1].point.z);
            tf2::Vector3 current_point(waypoints[i].point.x, waypoints[i].point.y, waypoints[i].point.z);
            x_axis = (current_point - prev_point).normalized();
        }

        // Y축: X축과 Z축의 외적
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized(); // X축을 다시 계산하여 직교성을 유지

        // 회전 행렬 (X, Y, Z축을 기준으로)
        tf2::Matrix3x3 rotation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        // roll, pitch, yaw 계산
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        // 결과를 파일에 기록 (x, y, z, yaw, roll, pitch)
        file << x << ", " << y << ", " << z << ", " << yaw << ", " << roll << ", " << pitch << ", " << force << "\n";
    }

    // 파일 닫기
    file.close();
    std::cout << "Poses saved to " << filename << std::endl;
}

// 콜백 함수: 웨이포인트 보간, 노멀 계산 및 퍼블리시
void waypointsCallback(const nrs_path_planning::Waypoints::ConstPtr &msg, ros::Publisher &pub, double desired_interval, const Triangle_mesh &mesh)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // 원래의 포인트들 추출
    std::vector<geometry_msgs::Point> original_points;
    for (const auto &waypoint : msg->waypoints)
    {
        original_points.push_back(waypoint.point);
    }

<<<<<<< HEAD
    if (original_points.size() < 2)
    {
        ROS_WARN("Not enough waypoints for interpolation");
        return;
    }
=======
    // 보간된 포인트들로부터 웨이포인트 생성 (노멀 벡터 포함)
    std::vector<nrs_path_planning::Waypoint> interpolated_waypoints = convertToWaypoints(interpolated_points, mesh);
>>>>>>> 745743660d549b29ec120686157a19684c5f67e3

    // 첫 번째 점과 마지막 점에서 face normal vector 구하기
    geometry_msgs::Point start_point = original_points.front();
    geometry_msgs::Point end_point = original_points.back();

<<<<<<< HEAD
    // 첫 번째와 마지막 점에서 face의 normal vector 구하기
    Eigen::Vector3d start_normal = getFaceNormal(start_point, mesh);
    Eigen::Vector3d end_normal = getFaceNormal(end_point, mesh);

    // start_point를 첫 번째 normal 방향으로 0.2만큼 띄움
    geometry_msgs::Point start_interpolated;
    start_interpolated.x = start_point.x + 0.1 * start_normal.x();
    start_interpolated.y = start_point.y + 0.1 * start_normal.y();
    start_interpolated.z = start_point.z + 0.1 * start_normal.z();

    // end_point를 마지막 normal 방향으로 0.2만큼 띄움
    geometry_msgs::Point end_interpolated;
    end_interpolated.x = end_point.x + 0.1 * end_normal.x();
    end_interpolated.y = end_point.y + 0.1 * end_normal.y();
    end_interpolated.z = end_point.z + 0.1 * end_normal.z();

    // 세 구간의 포인트 설정: start_point -> waypoints[0], waypoints, waypoints[-1] -> end_point
    std::vector<geometry_msgs::Point> first_segment{start_interpolated, original_points.front()};
    std::vector<geometry_msgs::Point> last_segment{original_points.back(), end_interpolated};

    // 첫 번째 구간 interpolation
    std::vector<geometry_msgs::Point> first_interpolated = interpolatePoints(first_segment, desired_interval, mesh);

    // 두 번째 구간 interpolation (원래 waypoints)
    std::vector<geometry_msgs::Point> middle_interpolated = interpolatePoints(original_points, desired_interval, mesh);

    // 세 번째 구간 interpolation
    std::vector<geometry_msgs::Point> last_interpolated = interpolatePoints(last_segment, desired_interval, mesh);
=======
    // 보간된 포인트들로부터  smoothing normal 웨이포인트 생성 (노멀 벡터 포함)
    std::vector<nrs_path_planning::Waypoint> smoothing_interpolated_waypoints = generateWaypointsWithNormals(interpolated_points, mesh);

    savePosesToFile(smoothing_interpolated_waypoints, "/home/nrs/catkin_ws/src/nrs_path_planning/data/waypoints_with_RPY.txt");

    // 새로운 웨이포인트 메시지 생성 및 퍼블리시
    nrs_path_planning::Waypoints interpolated_waypoints_msg;
    interpolated_waypoints_msg.waypoints = smoothing_interpolated_waypoints;
>>>>>>> 745743660d549b29ec120686157a19684c5f67e3

    // 첫 번째 구간의 normal vector 설정 (첫 번째 waypoints[0]의 normal과 동일하게 설정)
    std::vector<nrs_vision_rviz::Waypoint> first_waypoints = convertToWaypoints(first_interpolated, mesh);
    for (auto &waypoint : first_waypoints)
    {
        waypoint.normal.x = msg->waypoints[0].normal.x;
        waypoint.normal.y = msg->waypoints[0].normal.y;
        waypoint.normal.z = msg->waypoints[0].normal.z;
    }

    // 두 번째 구간의 normal vector는 그대로 convertToWaypoints 사용
    std::vector<nrs_vision_rviz::Waypoint> middle_waypoints = convertToWaypoints(middle_interpolated, mesh);

    // 세 번째 구간의 normal vector 설정 (마지막 waypoints[-1]의 normal과 동일하게 설정)
    std::vector<nrs_vision_rviz::Waypoint> last_waypoints = convertToWaypoints(last_interpolated, mesh);
    for (auto &waypoint : last_waypoints)
    {
        waypoint.normal.x = msg->waypoints.back().normal.x;
        waypoint.normal.y = msg->waypoints.back().normal.y;
        waypoint.normal.z = msg->waypoints.back().normal.z;
    }

    // 세 구간의 결과를 합침
    std::vector<nrs_vision_rviz::Waypoint> final_waypoints;
    final_waypoints.insert(final_waypoints.end(), first_waypoints.begin(), first_waypoints.end());
    final_waypoints.insert(final_waypoints.end(), middle_waypoints.begin(), middle_waypoints.end());
    final_waypoints.insert(final_waypoints.end(), last_waypoints.begin(), last_waypoints.end());

    // 첫 번째 구간 (start_point -> waypoints[0])
    saveFixedRPYPoseToFile(first_waypoints, "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/waypoints_with_RPY.txt", middle_waypoints[0], 0.0);

    // 두 번째 구간 (waypoints[0] -> waypoints[-1])
    savePosesToFile(middle_waypoints, "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/waypoints_with_RPY.txt", 5.0);

    // 세 번째 구간 (waypoints[-1] -> end_point)
    saveFixedRPYPoseToFile(last_waypoints, "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/waypoints_with_RPY.txt", middle_waypoints[middle_waypoints.size() - 2], 0.0);

    nrs_vision_rviz::Waypoints interpolated_waypoints_msg;
    interpolated_waypoints_msg.waypoints = final_waypoints;
    pub.publish(interpolated_waypoints_msg);
<<<<<<< HEAD
=======
    // Save interpolated waypoints to a file
    saveWaypointsToFile(msg->waypoints, "/home/nrs/catkin_ws/src/nrs_path_planning/data/waypoints.txt");
    saveWaypointsToFile(interpolated_waypoints, "/home/nrs/catkin_ws/src/nrs_path_planning/data/interpolated_waypoints.txt");
    saveWaypointsToFile(smoothing_interpolated_waypoints, "/home/nrs/catkin_ws/src/nrs_path_planning/data/smoothing_interpolated_waypoints.txt");
>>>>>>> 745743660d549b29ec120686157a19684c5f67e3

    //saveWaypointsToFile(msg->waypoints, "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/waypoints.txt");
    //saveWaypointsToFile(final_waypoints, "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/smoothing_interpolated_waypoints.txt");

    ROS_INFO("Published %lu final waypoints", final_waypoints.size());

    // 소요 시간 측정 및 출력
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    std::cout << "interpolation & Normal smoothing time: " << duration << " s" << std::endl;
}