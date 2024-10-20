#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nrs_vision_rviz/Waypoint.h>
#include <nrs_vision_rviz/Waypoints.h>

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
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <atomic>   // For atomic flag
#include <Eigen/QR> // QR decomposition을 위한 헤더 추가

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
std::vector<geometry_msgs::Point> original_points;

// 전역 중단 플래그 (Atomic으로 설정)
std::atomic<bool> reset_flag(false);

struct WaypointsSegment
{
    int start_idx;
    int end_idx;
};

// STL 파일을 읽어 Triangle_mesh 객체로 변환하는 함수
bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh);

// Find closest [face and barycentric coordinate on Mesh] with points
bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &mesh);

// Calculate GeodesicDistance between two points
double computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh);

// 일정한 간격을 가진 웨이포인트 생성 함수
std::vector<geometry_msgs::Point> interpolatePoints(
    const std::vector<geometry_msgs::Point> &points,
    double desired_interval,
    const Triangle_mesh &mesh,
    int option);

// 서피스에서 노멀 벡터를 계산하고 웨이포인트로 변환하는 함수
nrs_vision_rviz::Waypoints convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const std::vector<geometry_msgs::Point> &reference_points, const Triangle_mesh &mesh, int option);

void clearFile(const std::string &file_path);

void saveWayPointsTOFile(const nrs_vision_rviz::Waypoints &waypoints, const std::string &filename, double force);

std::vector<geometry_msgs::Point> generate_segment(std::vector<geometry_msgs::Point> &original_points, int option, Triangle_mesh &mesh);

// 콜백 함수: 웨이포인트 보간, 노멀 계산 및 퍼블리시
void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg, ros::Publisher &pub, double desired_interval, const Triangle_mesh &mesh);

void keyboardCallback(const std_msgs::String::ConstPtr &msg);

// 중단 플래그를 체크하며 작업을 중단할지 확인하는 함수
bool shouldReset()
{
    return reset_flag.load();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "waypoints_interpolator");
    ros::NodeHandle nh;

    // 웨이포인트 간의 원하는 간격 (예: 0.05m 간격)
    double desired_interval;
    nh.param("desired_interval", desired_interval, 0.00009); // 파라미터 서버에서 간격을 가져오거나 기본값 0.05 사용

    // Triangle_mesh 로드 또는 생성
    Triangle_mesh mesh;
    std::ifstream stl_file("/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/fenda_wrap.stl"); // STL 파일 경로를 설정
    if (!stl_file || !read_stl_file(stl_file, mesh))
    {
        ROS_ERROR("Failed to load or process STL file.");
        return 1;
    }

    // 퍼블리셔 선언
    ros::Publisher interpolated_waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("interpolated_waypoints_with_normals", 10);

    // 웨이포인트 구독 및 콜백 함수 설정
    ros::Subscriber waypoints_sub = nh.subscribe<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10,
                                                                             boost::bind(waypointsCallback, _1, boost::ref(interpolated_waypoints_pub), desired_interval, boost::cref(mesh)));
    ros::Subscriber keyboard_sub = nh.subscribe("nrs_command", 10, keyboardCallback);
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

std::vector<geometry_msgs::Point> interpolatePoints(
    const std::vector<geometry_msgs::Point> &points,
    double desired_interval,
    const Triangle_mesh &mesh,
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
                return desired_interval / 12.0 + scale * (desired_interval - desired_interval / 12.0); // 점진적 변화
            }
            else if (current_distance > total_distance - transition_length) // 끝부분 3cm 구간
            {
                double remaining_distance = total_distance - current_distance;
                double scale = 0.5 * (1 - cos((remaining_distance / transition_length) * M_PI));     // 끝에서 다시 느려짐
                return desired_interval / 12.0 + scale * (desired_interval - desired_interval / 12.0); // 점진적 변화
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

// 콜백 함수: 웨이포인트 보간, 노멀 계산 및 퍼블리시
void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg, ros::Publisher &pub, double desired_interval, const Triangle_mesh &mesh)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // 원래의 포인트들 추출

    for (const auto &waypoint : msg->waypoints)
    {
        original_points.push_back(waypoint.pose.position);
    }

    if (original_points.size() < 2)
    {
        ROS_WARN("Not enough waypoints for interpolation");
        return;
    }
    // 접근 segment 생성
    std::vector<geometry_msgs::Point> approach_segment = generate_segment(original_points, 1, mesh);

    // 후퇴 segment 생성
    std::vector<geometry_msgs::Point> retreat_segment = generate_segment(original_points, 2, mesh);

    // 복귀 segment 생성
    std::vector<geometry_msgs::Point> home_segment = generate_segment(original_points, 3, mesh);

    //------------------------------------------------------------------------------------------------------------------

    // 시각화용 첫 번째 구간 interpolation
    std::vector<geometry_msgs::Point> visual_approach_interpolated = interpolatePoints(approach_segment, 0.001, mesh, 1);
    shouldReset();
    // 두 번째 구간 interpolation (원래 waypoints)
    std::vector<geometry_msgs::Point> visual_original_interpolated = interpolatePoints(original_points, 0.001, mesh, 1);
    shouldReset();
    // 시각화용 세 번째 구간 interpolation
    std::vector<geometry_msgs::Point> visual_retreat_interpolated = interpolatePoints(retreat_segment, 0.001, mesh, 1);
    shouldReset();

    // 첫 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints visual_approach_waypoints = convertToWaypoints(visual_approach_interpolated, visual_original_interpolated, mesh, 1);
    shouldReset();
    // 두 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints visual_original_waypoints = convertToWaypoints(visual_original_interpolated, visual_original_interpolated, mesh, 2);
    shouldReset();
    // 세 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints visual_retreat_waypoints = convertToWaypoints(visual_retreat_interpolated, visual_original_interpolated, mesh, 3);
    shouldReset();

    nrs_vision_rviz::Waypoints visual_final_waypoints;
    // 첫 번째 구간의 웨이포인트 추가 (approach)
    visual_final_waypoints.waypoints.insert(
        visual_final_waypoints.waypoints.end(),
        visual_approach_waypoints.waypoints.begin(),
        visual_approach_waypoints.waypoints.end());

    // 두 번째 구간의 웨이포인트 추가 (original)
    visual_final_waypoints.waypoints.insert(
        visual_final_waypoints.waypoints.end(),
        visual_original_waypoints.waypoints.begin(),
        visual_original_waypoints.waypoints.end());

    // 세 번째 구간의 웨이포인트 추가 (retreat)
    visual_final_waypoints.waypoints.insert(
        visual_final_waypoints.waypoints.end(),
        visual_retreat_waypoints.waypoints.begin(),
        visual_retreat_waypoints.waypoints.end());

    nrs_vision_rviz::Waypoints visual_waypoints_msg;
    visual_waypoints_msg = visual_final_waypoints;
    pub.publish(visual_waypoints_msg);
    ROS_INFO("Visualizing + Moveit Path Planning Available");

    //-----------------------------------------------------------------------------------------------------------------
    // 제어용 첫 번째 구간 interpolation
    std::vector<geometry_msgs::Point> control_approach_interpolated = interpolatePoints(approach_segment, desired_interval, mesh, 2);
    shouldReset();
    // 제어용 두 번째 구간 interpolation (원래 waypoints)
    std::vector<geometry_msgs::Point> control_original_interpolated = interpolatePoints(original_points, desired_interval, mesh, 2);
    shouldReset();
    // 제어용 세 번째 구간 interpolation
    std::vector<geometry_msgs::Point> control_retreat_interpolated = interpolatePoints(retreat_segment, desired_interval, mesh, 2);
    shouldReset();
    // 제어용 네 번째 구간 interpolation
    std::vector<geometry_msgs::Point> control_home_interpolated = interpolatePoints(home_segment, desired_interval, mesh, 2);
    shouldReset();
    //------------------------------------------------------------------------------------------------------------------

    // 첫 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_approach_waypoints = convertToWaypoints(control_approach_interpolated, control_original_interpolated, mesh, 1);
    shouldReset();
    // 두 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_original_waypoints = convertToWaypoints(control_original_interpolated, control_original_interpolated, mesh, 2);
    shouldReset();
    // 세 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_retreat_waypoints = convertToWaypoints(control_retreat_interpolated, control_original_interpolated, mesh, 3);
    shouldReset();
    // 네 번째 구간의 quaternion 설정
    nrs_vision_rviz::Waypoints control_home_waypoints = convertToWaypoints(control_home_interpolated, control_original_interpolated, mesh, 4);
    shouldReset();

    // 파일 경로
    std::string file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/final_waypoints.txt";

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
}

void keyboardCallback(const std_msgs::String::ConstPtr &msg)
{

    if (msg->data == "reset")
    {
        original_points.clear();
        reset_flag.store(true); // 중단 플래그 설정
        ROS_INFO("Resetting operation, waypoints cleared.");
    }
    else
    {
        reset_flag.store(false);
    }
}