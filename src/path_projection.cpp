#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nrs_path/Waypoint.h>
#include <nrs_path/Waypoints.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
#include <CGAL/IO/STL_reader.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <boost/lexical_cast.hpp>
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
// 전역 설정 변수
double g_desired_interval = 0.00003;
double g_sampling_time = 0.002;
double g_starting_time = 3;
double g_last_resting_time = 3;
double g_acceleration_time = 1.0;
double g_time_counter = 0;

ros::Publisher g_interpolated_waypoints_pub;
ros::Publisher g_file_pub; // 전역 변수로 subscriber 선언
ros::Subscriber g_sub;
double g_projection_z = 0.5;

std::string g_mesh_file_path = "/home/nrs/catkin_ws/src/nrs_path/mesh/workpiece.stl";
std::string g_plane_path_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/Ori_path_transformed.txt";

// Basic Profiler Class
class Profiler
{
protected:
    std::vector<double> time;
    std::vector<double> data;
    double SamplingTime;
    double StartingTime;
    double LastRestingTime;
    double AccelerationTime;

public:
    // Constructor
    Profiler(const std::vector<double> &time, const std::vector<double> &data,
             double StartingTime, double LastRestingTime,
             double AccelerationTime, double SamplingTime);

    // Virtual Function
    virtual std::vector<std::vector<double>> AccDecProfiling() = 0;
};

// NRS Profiler Class (Corresponding the NRS_acc_dec_profiling)
class NRSProfiler : public Profiler
{
public:
    // Constructor
    NRSProfiler(const std::vector<double> &time, const std::vector<double> &data,
                double StartingTime, double LastRestingTime,
                double AccelerationTime, double SamplingTime);

    // NRS Acc-Dec Profiling
    std::vector<std::vector<double>> AccDecProfiling() override;
};

// Profiler Constructor
Profiler::Profiler(const std::vector<double> &time, const std::vector<double> &data,
                   double StartingTime, double LastRestingTime,
                   double AccelerationTime, double SamplingTime)
    : time(time), data(data), StartingTime(StartingTime),
      LastRestingTime(LastRestingTime), AccelerationTime(AccelerationTime),
      SamplingTime(SamplingTime) {}

// NRSProfiler Constructor
NRSProfiler::NRSProfiler(const std::vector<double> &time, const std::vector<double> &data,
                         double StartingTime, double LastRestingTime,
                         double AccelerationTime, double SamplingTime)
    : Profiler(time, data, StartingTime, LastRestingTime, AccelerationTime, SamplingTime) {}

// NRS Acc-Dcc Profiling
std::vector<std::vector<double>> NRSProfiler::AccDecProfiling()
{
    std::vector<std::vector<double>> Final_pos_interval;
    std::vector<double> Target_velocity(time.size(), 0.0);
    double Ti = StartingTime;
    double Ta = AccelerationTime;
    double Ts = SamplingTime;
    double Tl = LastRestingTime;
    double Tf = Ti + Ts * data.size() + Tl;

    // Target Velocity Calculation
    for (size_t j = 1; j < data.size(); j++)
    {
        Target_velocity[j] = (data[j] - data[j - 1]) / Ts;
    }

    std::vector<double> t;
    for (double i = 0; i <= Tf; i += Ts)
    {
        t.push_back(i);
    }

    // Interpolation
    std::vector<std::vector<double>> Interpolated(t.size(), std::vector<double>(2, 0.0));
    size_t Last_flag = 0;
    for (size_t i = 0; i < t.size(); i++)
    {
        if (t[i] <= Ti)
        {
            Interpolated[i] = {t[i], 0};
            Last_flag++;
        }
        else if (t[i] <= Ti + Ts * data.size())
        {
            Interpolated[i] = {t[i], Target_velocity[i - Last_flag]};
        }
        else
        {
            Interpolated[i] = {t[i], 0};
        }
    }

    // Velocity Profiling
    double m = Ta / Ts;
    std::vector<std::vector<double>> Final(t.size(), std::vector<double>(2, 0.0));
    Final_pos_interval.push_back({time[0], data[0]});

    for (size_t i = 1; i < t.size(); i++)
    {
        if (i <= m)
        {
            Final[i] = {t[i], Final[i - 1][1] + (Interpolated[i][1] - Interpolated[0][1]) / (i)};
        }
        else
        {
            Final[i] = {t[i], Final[i - 1][1] + (Interpolated[i][1] - Interpolated[i - (int)m][1]) / m};
        }
        Final_pos_interval.push_back({t[i], Final_pos_interval[i - 1][1] + Final[i][1] * Ts});
    }

    return Final_pos_interval;
}

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

    return max_z; // 가장 높은 z 값보다 5cm 위로 설정
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
            // std::cerr << "No intersection found for point: " << p << std::endl;
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

        home_position.x = 0.568;
        home_position.y = 0.318;
        home_position.z = 0.326;
        std::vector<geometry_msgs::Point> home_segment{end_retreat, home_position};

        return home_segment;
    }
}

// geometry_msgs::Point를 CGAL::Point_3로 변환하는 헬퍼 함수
std::vector<Point_3> convertToCGALPoints(const std::vector<geometry_msgs::Point> &pts)
{
    std::vector<Point_3> cgal_points;
    for (const auto &p : pts)
    {
        cgal_points.push_back(Point_3(p.x, p.y, p.z));
    }
    return cgal_points;
}

// CGAL::Point_3를 geometry_msgs::Point로 변환하는 헬퍼 함수
geometry_msgs::Point convertToROSPoint(const Point_3 &p)
{
    geometry_msgs::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    return pt;
}

void quaternionToRPY(double qx, double qy, double qz, double qw,
                     double &roll, double &pitch, double &yaw)
{
    // 쿼터니언 정규화
    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm == 0)
        norm = 1;
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;

    // roll (x축 회전)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double raw_roll = std::atan2(sinr_cosp, cosr_cosp);
    if (raw_roll < -2.5)
        roll = raw_roll + 2 * M_PI;
    else
        roll = raw_roll;
    // pitch (y축 회전)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::fabs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0, sinp); // 범위를 벗어나면 ±90도
    else
        pitch = std::asin(sinp);

    // yaw (z축 회전)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
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

    if (option == 0)
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

        // 3. 보간된 점들을 CGAL::Point_3 타입으로 변환
        std::vector<Point_3> interpolated_cgal_points = convertToCGALPoints(interpolated_points);
        // 4. projectPathOntoMesh 함수를 이용하여 보간된 점들을 메쉬 표면에 투영
        std::vector<Point_3> projected_points = projectPathOntoMesh(interpolated_cgal_points, tree);
        // 5. 투영된 CGAL 포인트들을 geometry_msgs::Point로 변환하여 반환
        std::vector<geometry_msgs::Point> final_points;
        for (const auto &p : projected_points)
        {
            final_points.push_back(convertToROSPoint(p));
        }
        return final_points;
    }

    else if (option == 1)
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
        return interpolated_points;
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
        return interpolated_points;
    }
}

nrs_path::Waypoints convertToWaypoints(const std::vector<geometry_msgs::Point> &points,
                                       const std::vector<geometry_msgs::Point> &reference_points,
                                       const Triangle_mesh &mesh, int option)
{
    nrs_path::Waypoints final_waypoints;

    Point_3 start_point(reference_points.begin()->x, reference_points.begin()->y, reference_points.begin()->z);
    Point_3 end_point(reference_points[reference_points.size() - 1].x,
                      reference_points[reference_points.size() - 1].y,
                      reference_points[reference_points.size() - 1].z);

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
        // tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        tf2::Vector3 y_axis = z_axis.cross((x_axis.normalized()));
        x_axis = y_axis.cross(z_axis).normalized();
        //  회전 행렬 계산
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion q;
        orientation_matrix.getRotation(q);

        for (auto &point : points)
        {
            nrs_path::Waypoint waypoint;
            waypoint.x = point.x;
            waypoint.y = point.y;
            waypoint.z = point.z;

            waypoint.qw = q.getW();
            waypoint.qx = q.getX();
            waypoint.qy = q.getY();
            waypoint.qz = q.getZ();
            final_waypoints.waypoints.push_back(waypoint);
        }
    }
    else if (option == 2) // original_segment (각 점마다 orientation 계산)
    {
        bool first = true;
        double prev_roll = 0.0, prev_pitch = 0.0, prev_yaw = 0.0;

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

            // tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
            tf2::Vector3 y_axis = z_axis.cross((x_axis.normalized()));
            x_axis = y_axis.cross(z_axis).normalized();
            //  회전 행렬 계산
            tf2::Matrix3x3 orientation_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z());
            tf2::Quaternion q;
            orientation_matrix.getRotation(q);

            nrs_path::Waypoint waypoint;
            waypoint.x = point.x;
            waypoint.y = point.y;
            waypoint.z = point.z;
            waypoint.qw = q.getW();
            waypoint.qx = q.getX();
            waypoint.qy = q.getY();
            waypoint.qz = q.getZ();
            waypoint.Fx = 0.0;
            waypoint.Fy = 0.0;
            waypoint.Fz = 10.0;

            final_waypoints.waypoints.push_back(waypoint);
        }
    }
    else if (option == 3) // retreat segment
    {
        tf2::Vector3 z_axis(-end_normal.x(), -end_normal.y(), -end_normal.z());
        z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        // tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        tf2::Vector3 y_axis = z_axis.cross((x_axis.normalized()));
        x_axis = y_axis.cross(z_axis).normalized();
        //  회전 행렬 계산
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion q;
        orientation_matrix.getRotation(q);

        for (auto &point : points)
        {
            nrs_path::Waypoint waypoint;
            waypoint.x = point.x;
            waypoint.y = point.y;
            waypoint.z = point.z;

            waypoint.qw = q.getW();
            waypoint.qx = q.getX();
            waypoint.qy = q.getY();
            waypoint.qz = q.getZ();
            final_waypoints.waypoints.push_back(waypoint);
        }
    }
    else if (option == 4) // home segment
    {
        tf2::Vector3 z_axis(-end_normal.x(), -end_normal.y(), -end_normal.z());
        z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        // tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        tf2::Vector3 y_axis = z_axis.cross((x_axis.normalized()));
        x_axis = y_axis.cross(z_axis).normalized();
        //  회전 행렬 계산
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion q;
        orientation_matrix.getRotation(q);

        for (auto &point : points)
        {
            nrs_path::Waypoint waypoint;
            waypoint.x = point.x;
            waypoint.y = point.y;
            waypoint.z = point.z;

            waypoint.qw = q.getW();
            waypoint.qx = q.getX();
            waypoint.qy = q.getY();
            waypoint.qz = q.getZ();
            final_waypoints.waypoints.push_back(waypoint);
        }
    }
    else if (option == 5)
    {
        bool first = true;
        double prev_roll = 0.0, prev_pitch = 0.0, prev_yaw = 0.0;

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

            // tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
            tf2::Vector3 y_axis = z_axis.cross((x_axis.normalized()));
            x_axis = y_axis.cross(z_axis).normalized();
            //  회전 행렬을 계산합니다.
            tf2::Matrix3x3 orientation_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z());
            tf2::Quaternion q;
            orientation_matrix.getRotation(q);

            nrs_path::Waypoint waypoint;
            waypoint.x = point.x;
            waypoint.y = point.y;
            waypoint.z = point.z;
            waypoint.qw = q.getW();
            waypoint.qx = q.getX();
            waypoint.qy = q.getY();
            waypoint.qz = q.getZ();
            waypoint.Fx = 0.0;
            waypoint.Fy = 0.0;
            waypoint.Fz = 10.0;

            final_waypoints.waypoints.push_back(waypoint);
        }
    }

    return final_waypoints;
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

// 파일 저장 함수
void saveWaypointsToFile(const nrs_path::Waypoints &control_final_waypoints,
                         const std::string &file_path)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << file_path << std::endl;
        return;
    }

    // 데이터 쓰기
    for (size_t i = 0; i < control_final_waypoints.waypoints.size(); ++i)
    {
        const auto &wp = control_final_waypoints.waypoints[i];

        tf2::Quaternion q(wp.qx, wp.qy, wp.qz, wp.qw);
        double roll, pitch, yaw;

        quaternionToRPY(wp.qx, wp.qy, wp.qz, wp.qw, roll, pitch, yaw);

        // 파일에 저장: x, y, z, roll, pitch, yaw, Fx, Fy, Fz (공백으로 구분)
        file << wp.x << " " << wp.y << " " << wp.z << " "
             << roll << " " << pitch << " " << yaw << " "
             << wp.Fx << " " << wp.Fy << " " << wp.Fz << " " << "\n";
    }

    file.close();
    std::cout << "Waypoints saved to " << file_path << std::endl;
}
std::vector<Point_3> readpathfile(const std::string &file_path, double &max_z, double desired_interval)
{
    // Step 1: Read original x,y values from file.
    std::vector<Point_3> original_points;
    std::ifstream infile(file_path);
    if (!infile.is_open())
    {
        std::cerr << "Error: Unable to open file: " << file_path << std::endl;
        return original_points;
    }

    std::string line;
    while (std::getline(infile, line))
    {
        if (line.empty())
            continue;
        std::istringstream iss(line);
        double x, y;
        if (!(iss >> x >> y))
        {
            std::cerr << "Error: Invalid line format: " << line << std::endl;
            continue;
        }
        // For each point, use max_z as the z value.
        original_points.push_back(Point_3(x, y, max_z));
    }
    infile.close();

    // If there are fewer than two points, interpolation is not needed.
    if (original_points.size() < 2)
        return original_points;

    // Step 2: Calculate cumulative distances along the original path.
    std::vector<double> cumulative_distances;
    cumulative_distances.push_back(0.0);
    for (size_t i = 1; i < original_points.size(); ++i)
    {
        double dx = original_points[i].x() - original_points[i - 1].x();
        double dy = original_points[i].y() - original_points[i - 1].y();
        double dz = original_points[i].z() - original_points[i - 1].z(); // dz is 0 because z == max_z for all points.
        double d = std::sqrt(dx * dx + dy * dy + dz * dz);
        cumulative_distances.push_back(cumulative_distances.back() + d);
    }

    double total_distance = cumulative_distances.back();

    // Step 3: Interpolate along the path at the desired interval.
    std::vector<Point_3> interpolated_points;
    interpolated_points.push_back(original_points.front());

    double current_distance = desired_interval;
    size_t seg = 1;
    while (current_distance < total_distance && seg < original_points.size())
    {
        // Find the segment such that cumulative_distances[seg-1] <= current_distance <= cumulative_distances[seg]
        while (seg < original_points.size() && cumulative_distances[seg] < current_distance)
            seg++;
        if (seg >= original_points.size())
            break;

        double d1 = cumulative_distances[seg - 1];
        double d2 = cumulative_distances[seg];
        double t = (current_distance - d1) / (d2 - d1);

        double x_interp = original_points[seg - 1].x() + t * (original_points[seg].x() - original_points[seg - 1].x());
        double y_interp = original_points[seg - 1].y() + t * (original_points[seg].y() - original_points[seg - 1].y());
        // z is set to max_z.
        double z_interp = max_z;

        interpolated_points.push_back(Point_3(x_interp, y_interp, z_interp));
        current_distance += desired_interval;
    }

    // Optionally, add the last original point if it was not already added.
    if (interpolated_points.back() != original_points.back())
        interpolated_points.push_back(original_points.back());

    return interpolated_points;
}

nrs_path::Waypoints ACCProfiling(const nrs_path::Waypoints &input_waypoints, double &sampling_time, double &starting_time, double &last_resting_time, double &acceleration_time, double &time_counter)
{
    nrs_path::Waypoints blendedPaths;
    std::vector<double> time, x, y, z, qw, qx, qy, qz;

    // 각 웨이포인트로부터 시간, 좌표, 쿼터니언 성분 추출
    for (size_t i = 0; i < input_waypoints.waypoints.size(); i++)
    {
        time.push_back(time_counter * sampling_time);
        x.push_back(input_waypoints.waypoints[i].x);
        y.push_back(input_waypoints.waypoints[i].y);
        z.push_back(input_waypoints.waypoints[i].z);
        qw.push_back(input_waypoints.waypoints[i].qw);
        qx.push_back(input_waypoints.waypoints[i].qx);
        qy.push_back(input_waypoints.waypoints[i].qy);
        qz.push_back(input_waypoints.waypoints[i].qz);

        time_counter++;
    }

    // 각 성분에 대해 프로파일러 생성
    NRSProfiler x_profiler(time, x, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler y_profiler(time, y, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler z_profiler(time, z, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qw_profiler(time, qw, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qx_profiler(time, qx, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qy_profiler(time, qy, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qz_profiler(time, qz, starting_time, last_resting_time, acceleration_time, sampling_time);

    // 각 성분의 가감속 프로파일링 결과 획득
    std::vector<std::vector<double>> x_result = x_profiler.AccDecProfiling();
    std::vector<std::vector<double>> y_result = y_profiler.AccDecProfiling();
    std::vector<std::vector<double>> z_result = z_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qw_result = qw_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qx_result = qx_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qy_result = qy_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qz_result = qz_profiler.AccDecProfiling();

    // 프로파일링 결과를 이용해 새로운 웨이포인트 생성
    for (int i = 0; i < x_result.size(); i++)
    {
        nrs_path::Waypoint blendedPath;

        blendedPath.x = x_result[i][1];
        blendedPath.y = y_result[i][1];
        blendedPath.z = z_result[i][1];
        blendedPath.qw = qw_result[i][1];
        blendedPath.qx = qx_result[i][1];
        blendedPath.qy = qy_result[i][1];
        blendedPath.qz = qz_result[i][1];

        // 선택 사항: 쿼터니언 보간 후 정규화 (정상적인 회전을 위해 권장)
        double norm = std::sqrt(blendedPath.qw * blendedPath.qw +
                                blendedPath.qx * blendedPath.qx +
                                blendedPath.qy * blendedPath.qy +
                                blendedPath.qz * blendedPath.qz);
        if (norm > 0)
        {
            blendedPath.qw /= norm;
            blendedPath.qx /= norm;
            blendedPath.qy /= norm;
            blendedPath.qz /= norm;
        }

        blendedPaths.waypoints.push_back(blendedPath);
    }

    return blendedPaths;
}

nrs_path::Waypoints control_ACCProfiling_RPY(const nrs_path::Waypoints &input_waypoints,
                                             int &approach_size,
                                             int &original_size,
                                             double force_input, // force value for original segment
                                             double &sampling_time,
                                             double &starting_time,
                                             double &last_resting_time,
                                             double &acceleration_time,
                                             double &time_counter,
                                             const std::string &output_file_path)
{
    nrs_path::Waypoints blendedPaths;

    // Vectors for time, spatial coordinates, quaternion components, and force
    std::vector<double> time, x, y, z;
    std::vector<double> qw, qx, qy, qz;
    std::vector<double> force;

    // Fill vectors: extract position and quaternion values from input waypoints.
    for (size_t i = 0; i < input_waypoints.waypoints.size(); i++)
    {
        time.push_back(time_counter * sampling_time);
        x.push_back(input_waypoints.waypoints[i].x);
        y.push_back(input_waypoints.waypoints[i].y);
        z.push_back(input_waypoints.waypoints[i].z);

        // Use quaternion values instead of rpy for profiling.
        qw.push_back(input_waypoints.waypoints[i].qw);
        qx.push_back(input_waypoints.waypoints[i].qx);
        qy.push_back(input_waypoints.waypoints[i].qy);
        qz.push_back(input_waypoints.waypoints[i].qz);

        // Force is applied only for the original segment.
        if (i >= static_cast<size_t>(approach_size) &&
            i < static_cast<size_t>(approach_size + original_size))
            force.push_back(force_input);
        else
            force.push_back(0.0);

        time_counter++;
    }

    // Create profilers for spatial coordinates, quaternion components, and force.
    NRSProfiler x_profiler(time, x, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler y_profiler(time, y, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler z_profiler(time, z, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qw_profiler(time, qw, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qx_profiler(time, qx, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qy_profiler(time, qy, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qz_profiler(time, qz, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler force_profiler(time, force, starting_time, last_resting_time, acceleration_time, sampling_time);

    // Run the profiling to obtain smooth profiles.
    std::vector<std::vector<double>> x_result = x_profiler.AccDecProfiling();
    std::vector<std::vector<double>> y_result = y_profiler.AccDecProfiling();
    std::vector<std::vector<double>> z_result = z_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qw_result = qw_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qx_result = qx_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qy_result = qy_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qz_result = qz_profiler.AccDecProfiling();
    std::vector<std::vector<double>> force_result = force_profiler.AccDecProfiling();

    // Open file for saving the control data.
    std::ofstream outfile(output_file_path);

    // Build the output waypoints and write each line to the file.
    // 파일 포맷: x y z roll pitch yaw 0.0 0.0 force_value
    // 여기서 roll, pitch, yaw는 프로파일링된 쿼터니언을 변환하여 계산합니다.
    for (int i = 0; i < x_result.size(); i++)
    {
        nrs_path::Waypoint blendedPath;
        blendedPath.x = x_result[i][1];
        blendedPath.y = y_result[i][1];
        blendedPath.z = z_result[i][1];
        blendedPath.qw = qw_result[i][1];
        blendedPath.qx = qx_result[i][1];
        blendedPath.qy = qy_result[i][1];
        blendedPath.qz = qz_result[i][1];

        // (선택 사항) 쿼터니언 정규화: 보간 후에는 정규화하여 올바른 회전을 보장합니다.
        double norm = std::sqrt(blendedPath.qw * blendedPath.qw +
                                blendedPath.qx * blendedPath.qx +
                                blendedPath.qy * blendedPath.qy +
                                blendedPath.qz * blendedPath.qz);
        if (norm > 0)
        {
            blendedPath.qw /= norm;
            blendedPath.qx /= norm;
            blendedPath.qy /= norm;
            blendedPath.qz /= norm;
        }

        blendedPaths.waypoints.push_back(blendedPath);

        // 쿼터니언을 roll, pitch, yaw로 변환하여 파일에 저장.
        tf2::Quaternion quat(blendedPath.qx, blendedPath.qy, blendedPath.qz, blendedPath.qw);
        double roll_out, pitch_out, yaw_out;

        quaternionToRPY(blendedPath.qx, blendedPath.qy, blendedPath.qz, blendedPath.qw, roll_out, pitch_out, yaw_out);

        outfile << blendedPath.x << " " << blendedPath.y << " " << blendedPath.z << " "
                << roll_out << " " << pitch_out << " " << yaw_out << " "
                << "0.0 0.0 " << force_result[i][1] << "\n";
    }
    outfile.close();
    ROS_INFO("Control data saved to: %s", output_file_path.c_str());
    return blendedPaths;
}

void control_ACCProfiling_RotationMatrix(const nrs_path::Waypoints &input_waypoints,
                                         int &approach_size,
                                         int &original_size,
                                         double force_input, // force value for original segment
                                         double &sampling_time,
                                         double &starting_time,
                                         double &last_resting_time,
                                         double &acceleration_time,
                                         double &time_counter,
                                         const std::string &output_file_path)
{
    nrs_path::Waypoints blendedPaths;

    // Vectors for time, spatial coordinates, quaternion components, and force
    std::vector<double> time, x, y, z;
    std::vector<double> qw, qx, qy, qz;
    std::vector<double> force;

    // Fill vectors: extract position and quaternion values from input waypoints.
    for (size_t i = 0; i < input_waypoints.waypoints.size(); i++)
    {
        time.push_back(time_counter * sampling_time);
        x.push_back(input_waypoints.waypoints[i].x);
        y.push_back(input_waypoints.waypoints[i].y);
        z.push_back(input_waypoints.waypoints[i].z);

        // Use quaternion values instead of rpy for profiling.
        qw.push_back(input_waypoints.waypoints[i].qw);
        qx.push_back(input_waypoints.waypoints[i].qx);
        qy.push_back(input_waypoints.waypoints[i].qy);
        qz.push_back(input_waypoints.waypoints[i].qz);

        // Force is applied only for the original segment.
        if (i >= static_cast<size_t>(approach_size) &&
            i < static_cast<size_t>(approach_size + original_size))
            force.push_back(force_input);
        else
            force.push_back(0.0);

        time_counter++;
    }

    // Create profilers for spatial coordinates, quaternion components, and force.
    NRSProfiler x_profiler(time, x, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler y_profiler(time, y, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler z_profiler(time, z, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qw_profiler(time, qw, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qx_profiler(time, qx, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qy_profiler(time, qy, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler qz_profiler(time, qz, starting_time, last_resting_time, acceleration_time, sampling_time);
    NRSProfiler force_profiler(time, force, starting_time, last_resting_time, acceleration_time, sampling_time);

    // Run the profiling to obtain smooth profiles.
    std::vector<std::vector<double>> x_result = x_profiler.AccDecProfiling();
    std::vector<std::vector<double>> y_result = y_profiler.AccDecProfiling();
    std::vector<std::vector<double>> z_result = z_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qw_result = qw_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qx_result = qx_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qy_result = qy_profiler.AccDecProfiling();
    std::vector<std::vector<double>> qz_result = qz_profiler.AccDecProfiling();
    std::vector<std::vector<double>> force_result = force_profiler.AccDecProfiling();

    // Open file for saving the control data.
    std::ofstream outfile(output_file_path);
    if (!outfile.is_open())
    {
        ROS_ERROR("Unable to open file: %s", output_file_path.c_str());
        return;
    }

    // Build the output waypoints and write each line to the file.
    // File format: x y z R11 R12 R13 R21 R22 R23 R31 R32 R33 0 0 force_value
    // The rotation matrix is computed from the interpolated quaternion.
    for (int i = 0; i < x_result.size(); i++)
    {
        nrs_path::Waypoint blendedPath;
        blendedPath.x = x_result[i][1];
        blendedPath.y = y_result[i][1];
        blendedPath.z = z_result[i][1];
        blendedPath.qw = qw_result[i][1];
        blendedPath.qx = qx_result[i][1];
        blendedPath.qy = qy_result[i][1];
        blendedPath.qz = qz_result[i][1];

        // Normalize the quaternion to ensure a valid rotation.
        double norm = std::sqrt(blendedPath.qw * blendedPath.qw +
                                blendedPath.qx * blendedPath.qx +
                                blendedPath.qy * blendedPath.qy +
                                blendedPath.qz * blendedPath.qz);
        if (norm > 0)
        {
            blendedPath.qw /= norm;
            blendedPath.qx /= norm;
            blendedPath.qy /= norm;
            blendedPath.qz /= norm;
        }

        blendedPaths.waypoints.push_back(blendedPath);

        // Convert the quaternion to a rotation matrix using tf2
        tf2::Quaternion quat(blendedPath.qx, blendedPath.qy, blendedPath.qz, blendedPath.qw);
        tf2::Matrix3x3 m(quat);

        tf2::Vector3 row0 = m.getRow(0);
        tf2::Vector3 row1 = m.getRow(1);
        tf2::Vector3 row2 = m.getRow(2);

        double r11 = row0.x();
        double r12 = row0.y();
        double r13 = row0.z();
        double r21 = row1.x();
        double r22 = row1.y();
        double r23 = row1.z();
        double r31 = row2.x();
        double r32 = row2.y();
        double r33 = row2.z();

        // Save the waypoint with rotation matrix values
        outfile << blendedPath.x << " " << blendedPath.y << " " << blendedPath.z << " "
                << r11 << " " << r12 << " " << r13 << " "
                << r21 << " " << r22 << " " << r23 << " "
                << r31 << " " << r32 << " " << r33 << " "
                << "0 0 " << force_result[i][1] << "\n";
    }
    outfile.close();
    ROS_INFO("Control data saved to: %s", output_file_path.c_str());
}

void saveACCProfiledWayPointsTOFile(const std::vector<std::vector<double>> &blended_path, const std::string &filename)
{
    // 파일 열기
    std::ofstream outfile(filename, std::ios_base::app);

    if (outfile.is_open())
    {
        for (const auto &row : blended_path)
        {
            for (const auto &col : row)
            {
                outfile << col << " ";
            }
            outfile << std::endl;
        }
        outfile.close();
    }
    else
    {
        std::cerr << "Can not open the file: " << filename << std::endl;
    }
}
// SLERP 헬퍼 함수 (tf2::Quaternion에 대한 간단한 SLERP 구현)
tf2::Quaternion customSlerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, double t)
{
    // 두 쿼터니언의 내적 계산
    double dot = q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w();
    tf2::Quaternion q2_copy = q2;
    // 내적이 음수면 한 쿼터니언을 반전시켜 최단 경로로 보간
    if (dot < 0.0)
    {
        q2_copy = q2_copy.operator-();
        dot = -dot;
    }
    // 두 쿼터니언이 매우 가까우면 선형 보간
    if (dot > 0.9995)
    {
        tf2::Quaternion result = q1 * (1.0 - t) + q2_copy * t;
        result.normalize();
        return result;
    }
    // 일반적인 경우 SLERP 공식 적용
    double theta_0 = acos(dot); // 두 쿼터니언 사이의 각도
    double theta = theta_0 * t; // 보간 각도
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    double s1 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s2 = sin_theta / sin_theta_0;

    tf2::Quaternion result = (q1 * s1) + (q2_copy * s2);
    result.normalize();
    return result;
}

// 쿼터니언 기반 보간 함수: 위치는 선형 보간, orientation은 SLERP로 보간 후 Euler 각도로 변환.
nrs_path::Waypoints interpolatexyzrpy(const nrs_path::Waypoints &input, double desired_interval)
{
    nrs_path::Waypoints output;
    if (input.waypoints.empty())
        return output;

    // 누적 거리를 계산하기 위한 벡터 (위치 보간용)
    std::vector<double> cumulative_distances;
    cumulative_distances.push_back(0.0);

    // 각 웨이포인트 사이의 누적 거리를 계산
    for (size_t i = 1; i < input.waypoints.size(); i++)
    {
        const auto &p0 = input.waypoints[i - 1];
        const auto &p1 = input.waypoints[i];
        double dx = p1.x - p0.x;
        double dy = p1.y - p0.y;
        double dz = p1.z - p0.z;
        double segment_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        cumulative_distances.push_back(cumulative_distances.back() + segment_distance);
    }

    // 전체 경로 길이
    double total_distance = cumulative_distances.back();

    // 전체 경로 길이를 desired_interval로 나누어 보간점 생성
    for (double d = 0.0; d <= total_distance; d += desired_interval)
    {
        // 현재 보간점이 속하는 구간 찾기
        size_t i = 1;
        while (i < cumulative_distances.size() && cumulative_distances[i] < d)
            i++;

        // 구간 내 보간 비율 t 계산
        double t = (d - cumulative_distances[i - 1]) / (cumulative_distances[i] - cumulative_distances[i - 1]);

        // 선형 보간: 위치는 단순 선형 보간
        const auto &p0 = input.waypoints[i - 1];
        const auto &p1 = input.waypoints[i];
        nrs_path::Waypoint interp_wp;
        interp_wp.x = p0.x + t * (p1.x - p0.x);
        interp_wp.y = p0.y + t * (p1.y - p0.y);
        interp_wp.z = p0.z + t * (p1.z - p0.z);

        interp_wp.Fx = p0.Fx + t * (p1.Fx - p0.Fx);
        interp_wp.Fy = p0.Fy + t * (p1.Fy - p0.Fy);
        interp_wp.Fz = p0.Fz + t * (p1.Fz - p0.Fz);
        
        // orientation 보간: 입력 웨이포인트의 쿼터니언을 이용해 SLERP 수행
        tf2::Quaternion q0(p0.qx, p0.qy, p0.qz, p0.qw);
        tf2::Quaternion q1(p1.qx, p1.qy, p1.qz, p1.qw);
        tf2::Quaternion q_interp = customSlerp(q0, q1, t);

        interp_wp.qw = q_interp.getW();
        interp_wp.qx = q_interp.getX();
        interp_wp.qy = q_interp.getY();
        interp_wp.qz = q_interp.getZ();

        output.waypoints.push_back(interp_wp);
    }

    return output;
}
void sendFile(const std::string &file_path, ros::Publisher &file_pub)
{
    // 파일 열기
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    // 파일 데이터 읽기
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string file_data = buffer.str();

    // 파일을 닫기
    file.close();

    // 파일 데이터를 퍼블리시
    std_msgs::String msg;
    msg.data = file_data;
    ROS_INFO("Sending file data...");
    file_pub.publish(msg); // 퍼블리시
    ROS_INFO("File data sent.");
}

bool executePathProjectionCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Service called: Executing path projection and waypoint generation.");
    std::cout << "mesh data: " << g_mesh_file_path << std::endl;

    original_points.clear();
    g_projection_z = initializeMeshAndGetMaxZ(g_mesh_file_path, mesh, tree);

    // Convert the received waypoints to a vector of Point_3,
    // setting the z-value to g_projection_z (as done previously by readpathfile).
    std::vector<Point_3> path_2D = readpathfile(g_plane_path_file_path, g_projection_z, 0.001);
    ROS_INFO("Number of original points: %lu", path_2D.size());
    // Project the 2D path onto the 3D mesh.
    std::vector<Point_3> projected_points = projectPathOntoMesh(path_2D, tree);

    ROS_INFO("Path projection complete. Number of projected points: %lu", projected_points.size());

    auto start_time_point = std::chrono::high_resolution_clock::now();
    for (const auto &p : projected_points)
    {
        geometry_msgs::Point ros_point;
        ros_point.x = p.x();
        ros_point.y = p.y();
        ros_point.z = p.z();
        original_points.push_back(ros_point);
    }

    // 접근, 후퇴, 복귀 세그먼트 생성
    std::vector<geometry_msgs::Point> approach_segment = generate_segment(original_points, 1, mesh);
    std::vector<geometry_msgs::Point> retreat_segment = generate_segment(original_points, 2, mesh);
    std::vector<geometry_msgs::Point> home_segment = generate_segment(original_points, 3, mesh);

    // 시각화용 보간 (옵션 1)
    std::vector<geometry_msgs::Point> visual_approach_interpolated = interpolatePoints(approach_segment, 0.001, 1);
    std::vector<geometry_msgs::Point> visual_original_interpolated = interpolatePoints(original_points, 0.001, 1);
    std::vector<geometry_msgs::Point> visual_retreat_interpolated = interpolatePoints(retreat_segment, 0.001, 1);

    nrs_path::Waypoints visual_approach_waypoints = convertToWaypoints(visual_approach_interpolated, original_points, mesh, 1);
    nrs_path::Waypoints visual_original_waypoints = convertToWaypoints(visual_original_interpolated, original_points, mesh, 2);
    nrs_path::Waypoints visual_retreat_waypoints = convertToWaypoints(visual_retreat_interpolated, original_points, mesh, 3);

    nrs_path::Waypoints visual_final_waypoints;
    visual_final_waypoints.waypoints.insert(visual_final_waypoints.waypoints.end(),
                                            visual_approach_waypoints.waypoints.begin(), visual_approach_waypoints.waypoints.end());
    visual_final_waypoints.waypoints.insert(visual_final_waypoints.waypoints.end(),
                                            visual_original_waypoints.waypoints.begin(), visual_original_waypoints.waypoints.end());
    visual_final_waypoints.waypoints.insert(visual_final_waypoints.waypoints.end(),
                                            visual_retreat_waypoints.waypoints.begin(), visual_retreat_waypoints.waypoints.end());
    double accel = 0.05;
    visual_final_waypoints = ACCProfiling(visual_final_waypoints, g_sampling_time, g_starting_time,
                                          g_last_resting_time, accel, g_time_counter);

    // g_interpolated_waypoints_pub.publish(visual_final_waypoints);
    std::string visual_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/visual_final_waypoints.txt";
    clearFile(visual_file_path);
    int visual_approach_size = visual_approach_interpolated.size();
    int visual_original_size = visual_original_interpolated.size();
    saveWaypointsToFile(visual_final_waypoints, visual_file_path);

    // 제어용 보간 (옵션 2)
    double interval = 0.00005;
    std::vector<geometry_msgs::Point> control_approach_interpolated = interpolatePoints(approach_segment, interval, 2);
    std::vector<geometry_msgs::Point> control_original_interpolated = interpolatePoints(original_points, 0.001, 2);
    std::vector<geometry_msgs::Point> control_retreat_interpolated = interpolatePoints(retreat_segment, interval, 2);
    std::vector<geometry_msgs::Point> control_home_interpolated = interpolatePoints(home_segment, interval, 2);

    nrs_path::Waypoints control_approach_waypoints = convertToWaypoints(control_approach_interpolated, control_original_interpolated, mesh, 1);
    nrs_path::Waypoints control_original_waypoints = convertToWaypoints(control_original_interpolated, control_original_interpolated, mesh, 2);
    nrs_path::Waypoints control_retreat_waypoints = convertToWaypoints(control_retreat_interpolated, control_original_interpolated, mesh, 3);
    nrs_path::Waypoints control_home_waypoints = convertToWaypoints(control_home_interpolated, control_original_interpolated, mesh, 4);
    // control_original_waypoints = interpolatexyzrpy(control_original_waypoints, g_desired_interval);
    nrs_path::Waypoints control_final_waypoints;
    control_final_waypoints.waypoints.insert(control_final_waypoints.waypoints.end(),
                                             control_approach_waypoints.waypoints.begin(), control_approach_waypoints.waypoints.end());
    control_final_waypoints.waypoints.insert(control_final_waypoints.waypoints.end(),
                                             control_original_waypoints.waypoints.begin(), control_original_waypoints.waypoints.end());
    control_final_waypoints.waypoints.insert(control_final_waypoints.waypoints.end(),
                                             control_retreat_waypoints.waypoints.begin(), control_retreat_waypoints.waypoints.end());
    control_final_waypoints.waypoints.insert(control_final_waypoints.waypoints.end(),
                                             control_home_waypoints.waypoints.begin(), control_home_waypoints.waypoints.end());
    control_final_waypoints = interpolatexyzrpy(control_final_waypoints, g_desired_interval);
    int approach_size = control_approach_waypoints.waypoints.size();
    int original_size = control_original_waypoints.waypoints.size();
    std::string RPY_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/final_waypoints_RPY.txt";
    std::string RotationMatrix_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/final_waypoints_RotationMatrix.txt";
    clearFile(RPY_file_path);
    clearFile(RotationMatrix_file_path);

    saveWaypointsToFile(control_final_waypoints, RPY_file_path);

    // control_final_waypoints = control_ACCProfiling_RPY(control_final_waypoints, approach_size, original_size, 10.0, g_sampling_time, g_starting_time,
    //                                                    g_last_resting_time, g_acceleration_time, g_time_counter, RPY_file_path);
    // control_ACCProfiling_RotationMatrix(control_final_waypoints, approach_size, original_size, 10.0, g_sampling_time, g_starting_time,
    //                          g_last_resting_time, g_acceleration_time, g_time_counter, RotationMatrix_file_path);

    g_interpolated_waypoints_pub.publish(control_final_waypoints);

    std::cout << "original_size: " << original_size << std::endl;

    ROS_INFO("Saved %lu final waypoints", control_final_waypoints.waypoints.size());

    sendFile(RPY_file_path, g_file_pub); // 퍼블리시 함수를 호출하여 파일을 전송
    auto end_time_point = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_point - start_time_point).count();
    std::cout << "Interpolation & Normal smoothing time: " << duration << " s" << std::endl;

    return true;
}

// // [생성] : "/clicked_point" 토픽 콜백 함수 (경로 생성)
// void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
// {
//     std::cout << "-----------------------------------new waypoints coming---------------------------------" << std::endl;

//     Kernel::Point_3 clicked_point(msg->point.x, msg->point.y, msg->point.z);

//     face_descriptor face;
//     Surface_mesh_shortest_path::Barycentric_coordinates location;

//     if (!callback_handler.n_geodesic.locate_face_and_point(clicked_point, face, location, tmesh))
//     {
//         std::cerr << "Failed to locate the clicked point on the mesh." << std::endl;
//         return;
//     }

//     Kernel::Point_3 v1, v2, v3;
//     int i = 0;
//     for (auto v : vertices_around_face(tmesh.halfedge(face), tmesh))
//     {
//         if (i == 0)
//             v1 = tmesh.point(v);
//         else if (i == 1)
//             v2 = tmesh.point(v);
//         else if (i == 2)
//             v3 = tmesh.point(v);
//         ++i;
//     }

//     Kernel::Point_3 projected_point = CGAL::barycenter(v1, location[0], v2, location[1], v3, location[2]);

//     Eigen::Vector3d projected_point_eigen(projected_point.x(), projected_point.y(), projected_point.z());

//     callback_handler.selected_points.push_back(projected_point_eigen);
// }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_projection_node");
    ros::NodeHandle nh;
    // ros::Subscriber clicked_point_sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);

    // 퍼블리셔 전역 변수 초기화
    g_interpolated_waypoints_pub = nh.advertise<nrs_path::Waypoints>("interpolated_waypoints", 10);
    g_file_pub = nh.advertise<std_msgs::String>("path_publisher", 10);

    // 서비스 서버 광고
    ros::ServiceServer service = nh.advertiseService("projection", executePathProjectionCallback);
    ROS_INFO("Path projection service node ready. Waiting for service calls...");
    ros::spin();
    return 0;
}