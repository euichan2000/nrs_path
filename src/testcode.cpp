#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nrs_vision_rviz/Waypoint.h>
#include <nrs_vision_rviz/Waypoints.h>
#include <fstream>

// CGAL 관련 헤더
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL_reader.h>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <map>
#include <chrono>
#include <std_msgs/String.h>
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

std::vector<nrs_vision_rviz::Waypoint> generateWaypointsWithNormals(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh &mesh)
{
    std::vector<nrs_vision_rviz::Waypoint> waypoints;

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
        nrs_vision_rviz::Waypoint waypoint_msg;
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

// 함수: Waypoints를 입력받아 x, y, z, yaw, roll, pitch 값을 계산하고 텍스트 파일에 저장
void saveWaypointsRPYToFile(const std::vector<nrs_vision_rviz::Waypoint> &waypoints, const std::string &filename, double force)
{
    // 파일 열기 (append 모드로 열어서 내용을 추가)
    std::ofstream file(filename, std::ios_base::app); // 'app' 모드로 파일에 내용을 추가
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    // 소수점 여섯째자리까지 출력하도록 설정
    file << std::fixed << std::setprecision(6);

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
            x_axis = (current_point - next_point).normalized(); // 방향 벡터를 단위 벡터로 정규화
        }
        else
        {
            // 마지막 웨이포인트는 이전 웨이포인트와의 차이로 방향을 설정
            tf2::Vector3 prev_point(waypoints[i - 1].point.x, waypoints[i - 1].point.y, waypoints[i - 1].point.z);
            tf2::Vector3 current_point(waypoints[i].point.x, waypoints[i].point.y, waypoints[i].point.z);
            x_axis = (prev_point - current_point).normalized();
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

        // 고정된 roll, pitch, yaw 값으로 파일에 기록
        file << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << "0.000000 " << "0.000000 " << force << "\n";
    }

    // 파일 닫기
    file.close();
    std::cout << "Poses saved to " << filename << std::endl;
}

// 콜백 함수: 웨이포인트 보간, 노멀 계산 및 퍼블리시
void waypointsCallback(const nrs_vision_rviz::Waypoints::ConstPtr &msg, ros::Publisher &pub, double desired_interval, const Triangle_mesh &mesh)
{
    // 원래 웨이포인트들 추출
    std::vector<geometry_msgs::Point> original_points;
    for (const auto &waypoint : msg->waypoints)
    {
        original_points.push_back(waypoint.point);
    }

    if (original_points.size() < 2)
    {
        ROS_WARN("Not enough waypoints for interpolation");
        return;
    }

    // 중간 웨이포인트들을 보간
    std::vector<geometry_msgs::Point> middle_interpolated = interpolatePoints(original_points, desired_interval, mesh);

    // 보간된 웨이포인트들의 노멀 벡터를 계산
    std::vector<nrs_vision_rviz::Waypoint> middle_waypoints = generateWaypointsWithNormals(middle_interpolated, mesh);

    // 파일 경로
    std::string file_path = "/home/nrs/catkin_ws/src/nrs_vision_rviz/data/test.txt";

    // 두 번째 구간 (waypoints[0] -> waypoints[-1])
    saveWaypointsRPYToFile(middle_waypoints, file_path, 5.0);

    nrs_vision_rviz::Waypoints interpolated_waypoints_msg;
    interpolated_waypoints_msg.waypoints = middle_waypoints;
    pub.publish(interpolated_waypoints_msg);

    ROS_INFO("Published %lu interpolated waypoints with normals", middle_waypoints.size());
}
// 첫 번째 및 세 번째 구간: 고정된 RPY 값을 사용하여 텍스트 파일에 저장
void saveWaypointsRPY_Down_ToFile(const std::vector<nrs_vision_rviz::Waypoint> &waypoints, const std::string &filename, const std::vector<nrs_vision_rviz::Waypoint> &reference_waypoints, double force)
{
    // Z축: 웨이포인트의 normal vector (법선 벡터)
    tf2::Vector3 z_axis(waypoints[0].normal.x, waypoints[0].normal.y, waypoints[0].normal.z);
    z_axis.normalize(); // 법선 벡터를 단위 벡터로 정규화

    tf2::Vector3 x_axis;
    if (waypoints.size() > 1)
    {
        // 첫 번째 웨이포인트와 두 번째 웨이포인트 사이의 방향 벡터 계산
        // tf2::Vector3 next_point(reference_waypoints[1].point.x, reference_waypoints[1].point.y, reference_waypoints[1].point.z);
        // tf2::Vector3 current_point(reference_waypoints[0].point.x, reference_waypoints[0].point.y, reference_waypoints[0].point.z);
        // x_axis = (current_point - next_point).normalized();
        x_axis.setValue(-1.0, -1.0, 0.0);
    }
    else
    {
        // 점이 하나만 있는 경우 진행 방향은 임의로 설정 (예: X축 방향)
        x_axis.setValue(-1.0, -1.0, 0.0);
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
    // 소수점 첫째자리까지 출력하도록 설정
    file << std::fixed << std::setprecision(6);

    // 고정된 RPY 값을 모든 웨이포인트에 적용
    for (const auto &waypoint : waypoints)
    {
        // 현재 웨이포인트의 위치 (x, y, z)
        double x = waypoint.point.x;
        double y = waypoint.point.y;
        double z = waypoint.point.z;

        // 고정된 roll, pitch, yaw 값으로 파일에 기록
        file << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << "0.000000 " << "0.000000 " << force << "\n";
    }

    // 파일 닫기
    file.close();
    std::cout << "Fixed RPY Poses saved to " << filename << std::endl;
}

std::vector<geometry_msgs::Point> generate_segment(std::vector<geometry_msgs::Point> &original_points, int &option)
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

    if (option == 1)
    {
        start_approach.x = start_point.x + 0.1 * start_normal.x();
        start_approach.y = start_point.y + 0.1 * start_normal.y();
        start_approach.z = start_point.z + 0.1 * start_normal.z();
        std::vector<geometry_msgs::Point> first_segment{start_approach, original_points.front()};
        return first_segment;
    }
    else if (option == 2)
    {
        end_retreat.x = end_point.x + 0.1 * end_normal.x();
        end_retreat.y = end_point.y + 0.1 * end_normal.y();
        end_retreat.z = end_point.z + 0.1 * end_normal.z();
        std::vector<geometry_msgs::Point> last_segment{original_points.back(), end_retreat};
        return last_segment;
    }
    else if (option == 3)
    {

        end_retreat.x = end_point.x + 0.1 * end_normal.x();
        end_retreat.y = end_point.y + 0.1 * end_normal.y();
        end_retreat.z = end_point.z + 0.1 * end_normal.z();

        home_position.x = 0.55;
        home_position.y = 0.09;
        home_position.z = 0.6;
        std::vector<geometry_msgs::Point> home_segment{end_retreat, home_position};
        return home_segment;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testcode");
    ros::NodeHandle nh;

    // 웨이포인트 간의 원하는 간격 (예: 0.05m 간격)
    double desired_interval;
    nh.param("desired_interval", desired_interval, 0.001); // 파라미터 서버에서 간격을 가져오거나 기본값 사용

    // Triangle_mesh 로드 또는 생성
    Triangle_mesh mesh;
    std::ifstream stl_file("/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/fenda_wrap.stl"); // STL 파일 경로를 설정
    if (!stl_file || !read_stl_file(stl_file, mesh))
    {
        ROS_ERROR("Failed to load or process STL file.");
        return 1;
    }

    // 퍼블리셔 선언
    ros::Publisher interpolated_waypoints_pub = nh.advertise<nrs_vision_rviz::Waypoints>("interpolation_waypoints_with_normals", 10);

    // 웨이포인트 구독 및 콜백 함수 설정
    ros::Subscriber waypoints_sub = nh.subscribe<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10,
                                                                             boost::bind(waypointsCallback, _1, boost::ref(interpolated_waypoints_pub), desired_interval, boost::cref(mesh)));

    ros::spin();

    return 0;
}
