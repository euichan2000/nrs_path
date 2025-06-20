#include "nrs_interpolation.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/locate.h>

// 세그먼트 생성 함수 구현
std::vector<geometry_msgs::Point> nrs_interpolation::generate_segment(std::vector<geometry_msgs::Point> &original_points,
                                                                      int option,
                                                                      const Triangle_mesh &mesh)
{
    geometry_msgs::Point start_point = original_points.front();
    geometry_msgs::Point end_point = original_points.back();
    // 첫 번째와 마지막 점에서 face의 normal vector 구하기
    Eigen::Vector3d start_normal = getFaceNormal(start_point, mesh);
    Eigen::Vector3d end_normal = getFaceNormal(end_point, mesh);
    geometry_msgs::Point start_approach;
    geometry_msgs::Point end_retreat;
    if (option == 1) // approach
    {

        start_approach.x = start_point.x + start_normal.x() * 0.1;
        start_approach.y = start_point.y + start_normal.y() * 0.1;
        start_approach.z = start_point.z + start_normal.z() * 0.1;

        return {start_approach, start_point};
    }
    else if (option == 2) // retreat
    {

        end_retreat.x = end_point.x + 0.1 * end_normal.x();
        end_retreat.y = end_point.y + 0.1 * end_normal.y();
        end_retreat.z = end_point.z + 0.1 * end_normal.z();

        return {end_point, end_retreat};
    }
    else if (option == 3) // home
    {
        end_retreat.x = end_point.x + 0.1 * end_normal.x();
        end_retreat.y = end_point.y + 0.1 * end_normal.y();
        end_retreat.z = end_point.z + 0.1 * end_normal.z();
        geometry_msgs::Point home;
        home.x = 0.573;
        home.y = -0.127;
        home.z = 0.25;
        return {end_retreat, home};
    }
    return {};
}
std::vector<double> nrs_interpolation::computeCumulativeDistances(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<double> cumulative_distances(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i)
    {
        Eigen::Vector3d p0(points[i - 1].x, points[i - 1].y, points[i - 1].z);
        Eigen::Vector3d p1(points[i].x, points[i].y, points[i].z);
        cumulative_distances[i] = cumulative_distances[i - 1] + (p1 - p0).norm();
    }
    return cumulative_distances;
}

// Option 1: 일정 간격 보간
std::vector<geometry_msgs::Point> nrs_interpolation::interpolatePoints_Constant(const std::vector<geometry_msgs::Point> &points,
                                                                                const std::vector<double> &cumulative_distances,
                                                                                double desired_interval)
{
    std::vector<geometry_msgs::Point> interpolated_points;
    double current_distance = 0.0;
    size_t j = 1;
    while (j < points.size())
    {
        if (cumulative_distances[j] >= current_distance + desired_interval)
        {
            double t = (current_distance + desired_interval - cumulative_distances[j - 1]) /
                       (cumulative_distances[j] - cumulative_distances[j - 1]);

            geometry_msgs::Point ip;
            ip.x = points[j - 1].x + t * (points[j].x - points[j - 1].x);
            ip.y = points[j - 1].y + t * (points[j].y - points[j - 1].y);
            ip.z = points[j - 1].z + t * (points[j].z - points[j - 1].z);

            interpolated_points.push_back(ip);
            current_distance += desired_interval;
        }
        else
        {
            ++j;
        }
    }
    return interpolated_points;
}

// Option 2: 가변 간격 보간
std::vector<geometry_msgs::Point> nrs_interpolation::interpolatePoints_Variable(const std::vector<geometry_msgs::Point> &points,
                                                                                const std::vector<double> &cumulative_distances,
                                                                                double desired_interval)
{
    std::vector<geometry_msgs::Point> interpolated_points;
    double total_distance = cumulative_distances.back();
    double transition_length = 0.03; // 3cm

    auto computeVariableInterval = [&](double current_distance) -> double
    {
        if (current_distance < transition_length) // 초반 3cm 구간
        {
            double scale = 0.5 * (1 - cos((current_distance / transition_length) * M_PI));
            return desired_interval / 6.0 + scale * (desired_interval - desired_interval / 6.0);
        }
        else if (current_distance > total_distance - transition_length) // 끝부분 3cm 구간
        {
            double remaining_distance = total_distance - current_distance;
            double scale = 0.5 * (1 - cos((remaining_distance / transition_length) * M_PI));
            return desired_interval / 6.0 + scale * (desired_interval - desired_interval / 6.0);
        }
        else // 중간 구간은 일정 간격 유지
        {
            return desired_interval;
        }
    };

    double current_distance = 0.0;
    size_t j = 1;
    while (j < points.size())
    {
        double current_interval = computeVariableInterval(current_distance);

        if (cumulative_distances[j] >= current_distance + current_interval)
        {
            double t = (current_distance + current_interval - cumulative_distances[j - 1]) /
                       (cumulative_distances[j] - cumulative_distances[j - 1]);

            geometry_msgs::Point ip;
            ip.x = points[j - 1].x + t * (points[j].x - points[j - 1].x);
            ip.y = points[j - 1].y + t * (points[j].y - points[j - 1].y);
            ip.z = points[j - 1].z + t * (points[j].z - points[j - 1].z);

            interpolated_points.push_back(ip);
            current_distance += current_interval;
        }
        else
        {
            ++j;
        }
    }
    return interpolated_points;
}

std::vector<geometry_msgs::Point> nrs_interpolation::interpolatePoints(
    const std::vector<geometry_msgs::Point> &points,
    double desired_interval,
    int option)
{
    // 누적 거리 계산
    std::vector<double> cumulative_distances = computeCumulativeDistances(points);

    if (option == 1)
    {
        return interpolatePoints_Constant(points, cumulative_distances, desired_interval);
    }
    else if (option == 2)
    {
        return interpolatePoints_Variable(points, cumulative_distances, desired_interval);
    }
    else
    {
        return std::vector<geometry_msgs::Point>(); // 잘못된 옵션의 경우 빈 벡터 반환
    }
}

// 옵션 1: Approach Segment
// 시작점의 노멀(start_normal)을 기준으로 동일 orientation 적용
nrs_path::Waypoints nrs_interpolation::setToolVectorApproach(
    const std::vector<geometry_msgs::Point> &points,
    const Triangle_mesh &mesh,
    const Kernel::Vector_3 &start_normal)
{
    nrs_path::Waypoints waypoints;
    tf2::Vector3 z_axis(-start_normal.x(), -start_normal.y(), -start_normal.z());
    z_axis.normalize();
    tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
    tf2::Vector3 y_axis = z_axis.cross(x_axis.normalized());
    x_axis = y_axis.cross(z_axis).normalized();
    tf2::Matrix3x3 orientation_matrix(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion q;
    orientation_matrix.getRotation(q);

    for (const auto &point : points)
    {
        nrs_path::Waypoint wp;
        wp.x = point.x;
        wp.y = point.y;
        wp.z = point.z;
        wp.qw = q.getW();
        wp.qx = q.getX();
        wp.qy = q.getY();
        wp.qz = q.getZ();
        waypoints.waypoints.push_back(wp);
    }
    return waypoints;
}

//--------------------------------------------------------
// 옵션 2: Original Segment
// 각 점마다 해당 점이 속한 face를 찾아, 바리센트릭 좌표를 이용해 보간된 노멀로 orientation 계산
nrs_path::Waypoints nrs_interpolation::setToolVectorOriginal(
    const std::vector<geometry_msgs::Point> &points,
    const Triangle_mesh &mesh,
    double Fx, double Fy, double Fz)
{
    nrs_path::Waypoints waypoints;
    for (const auto &point : points)
    {
        Point_3 cgal_point(point.x, point.y, point.z);
        face_descriptor face;
        CGAL::cpp11::array<double, 3> location;
        if (!n_geodesic.locate_face_and_point(cgal_point, face, location, mesh))
        {
            ROS_ERROR("Failed to locate face for point: [%f, %f, %f]", point.x, point.y, point.z);
            continue;
        }

        // face의 세 정점을 가져와서 노멀 계산
        Triangle_mesh::Halfedge_index h = mesh.halfedge(face);
        vertex_descriptor v0 = mesh.source(h);
        vertex_descriptor v1 = mesh.target(h);
        vertex_descriptor v2 = mesh.target(mesh.next(h));
        Kernel::Vector_3 normal_v0 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v0, mesh);
        Kernel::Vector_3 normal_v1 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v1, mesh);
        Kernel::Vector_3 normal_v2 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v2, mesh);
        Kernel::Vector_3 interpolated_normal = location[0] * normal_v0 +
                                               location[1] * normal_v1 +
                                               location[2] * normal_v2;
        tf2::Vector3 z_axis(-interpolated_normal.x(), -interpolated_normal.y(), -interpolated_normal.z());
        z_axis.normalize();
        tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis.normalized());
        x_axis = y_axis.cross(z_axis).normalized();
        tf2::Matrix3x3 orientation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());
        tf2::Quaternion q;
        orientation_matrix.getRotation(q);

        nrs_path::Waypoint wp;
        wp.x = point.x;
        wp.y = point.y;
        wp.z = point.z;
        wp.qw = q.getW();
        wp.qx = q.getX();
        wp.qy = q.getY();
        wp.qz = q.getZ();
        wp.Fx = Fx;
        wp.Fy = Fy;
        wp.Fz = Fz;
        waypoints.waypoints.push_back(wp);
    }
    return waypoints;
}

//--------------------------------------------------------
// 옵션 3: Retreat Segment
// 마지막 reference point의 노멀(end_normal)을 기준으로 동일 orientation 적용
nrs_path::Waypoints nrs_interpolation::setToolVectorRetreat(
    const std::vector<geometry_msgs::Point> &points,
    const Triangle_mesh &mesh,
    const Kernel::Vector_3 &end_normal)
{
    nrs_path::Waypoints waypoints;
    tf2::Vector3 z_axis(-end_normal.x(), -end_normal.y(), -end_normal.z());
    z_axis.normalize();
    tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
    tf2::Vector3 y_axis = z_axis.cross(x_axis.normalized());
    x_axis = y_axis.cross(z_axis).normalized();
    tf2::Matrix3x3 orientation_matrix(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion q;
    orientation_matrix.getRotation(q);

    for (const auto &point : points)
    {
        nrs_path::Waypoint wp;
        wp.x = point.x;
        wp.y = point.y;
        wp.z = point.z;
        wp.qw = q.getW();
        wp.qx = q.getX();
        wp.qy = q.getY();
        wp.qz = q.getZ();
        waypoints.waypoints.push_back(wp);
    }
    return waypoints;
}

//--------------------------------------------------------
// 옵션 4: Home Segment
// 마지막 reference point의 노멀(end_normal)을 기준으로 계산한 orientation으로 전체 구간에 동일 적용
nrs_path::Waypoints nrs_interpolation::setToolVectorHome(
    const std::vector<geometry_msgs::Point> &points,
    const Triangle_mesh &mesh,
    const Kernel::Vector_3 &end_normal)
{
    nrs_path::Waypoints waypoints;
    tf2::Vector3 z_axis(-end_normal.x(), -end_normal.y(), -end_normal.z());
    z_axis.normalize();
    tf2::Vector3 x_axis(-1.0, -1.0, 0.0);
    tf2::Vector3 y_axis = z_axis.cross(x_axis.normalized());
    x_axis = y_axis.cross(z_axis).normalized();
    tf2::Matrix3x3 orientation_matrix(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion q;
    orientation_matrix.getRotation(q);

    for (const auto &point : points)
    {
        nrs_path::Waypoint wp;
        wp.x = point.x;
        wp.y = point.y;
        wp.z = point.z;
        wp.qw = q.getW();
        wp.qx = q.getX();
        wp.qy = q.getY();
        wp.qz = q.getZ();
        waypoints.waypoints.push_back(wp);
    }
    return waypoints;
}

//--------------------------------------------------------
nrs_path::Waypoints nrs_interpolation::setToolVectorFaceNormal(
    const std::vector<geometry_msgs::Point> &points,
    const Triangle_mesh &mesh,
    double Fx, double Fy, double Fz)
{
    nrs_path::Waypoints waypoints;
    for (const auto &point : points)
    {
        Point_3 cgal_point(point.x, point.y, point.z);
        face_descriptor face;
        CGAL::cpp11::array<double, 3> location;
        if (!n_geodesic.locate_face_and_point(cgal_point, face, location, mesh))
        {
            ROS_ERROR("Failed to locate face for point: [%f, %f, %f]",
                      point.x, point.y, point.z);
            continue;
        }
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

        tf2::Quaternion q;
        orientation_matrix.getRotation(q);

        // Waypoint 생성
        nrs_path::Waypoint wp;
        wp.x = point.x;
        wp.y = point.y;
        wp.z = point.z;
        wp.qw = q.getW();
        wp.qx = q.getX();
        wp.qy = q.getY();
        wp.qz = q.getZ();
        wp.Fx = Fx;
        wp.Fy = Fy;
        wp.Fz = Fz;
        waypoints.waypoints.push_back(wp);
    }
    return waypoints;
}

//--------------------------------------------------------
// 최종 wrapper 함수: reference_points로부터 시작/끝 노멀 계산 후 옵션에 따라 헬퍼 호출
nrs_path::Waypoints nrs_interpolation::setToolVector(const std::vector<geometry_msgs::Point> &approach_interpolated,
                                                     const std::vector<geometry_msgs::Point> &original_interpolated,
                                                     const std::vector<geometry_msgs::Point> &retreat_interpolated,
                                                     const std::vector<geometry_msgs::Point> &home_interpolated,
                                                     const Triangle_mesh &mesh,
                                                     double Fx, double Fy, double Fz)
{
    // reference_points에서 시작점과 끝점을 결정
    Point_3 start_point(original_interpolated.front().x, original_interpolated.front().y, original_interpolated.front().z);
    Point_3 end_point(original_interpolated.back().x, original_interpolated.back().y, original_interpolated.back().z);
    face_descriptor start_face, end_face;
    CGAL::cpp11::array<double, 3> start_location, end_location;
    n_geodesic.locate_face_and_point(start_point, start_face, start_location, mesh);
    n_geodesic.locate_face_and_point(end_point, end_face, end_location, mesh);
    Kernel::Vector_3 start_normal = CGAL::Polygon_mesh_processing::compute_face_normal(start_face, mesh);
    Kernel::Vector_3 end_normal = CGAL::Polygon_mesh_processing::compute_face_normal(end_face, mesh);

    nrs_path::Waypoints approach_waypoints = setToolVectorApproach(approach_interpolated, mesh, start_normal);
    nrs_path::Waypoints original_waypoints = setToolVectorOriginal(original_interpolated, mesh, Fx, Fy, Fz);
    nrs_path::Waypoints retreat_waypoints = setToolVectorRetreat(retreat_interpolated, mesh, end_normal);
    nrs_path::Waypoints home_waypoints = setToolVectorHome(home_interpolated, mesh, end_normal);

    nrs_path::Waypoints final_waypoints;
    final_waypoints.waypoints.insert(final_waypoints.waypoints.end(),
                                     approach_waypoints.waypoints.begin(), approach_waypoints.waypoints.end());
    final_waypoints.waypoints.insert(final_waypoints.waypoints.end(),
                                     original_waypoints.waypoints.begin(), original_waypoints.waypoints.end());
    final_waypoints.waypoints.insert(final_waypoints.waypoints.end(),
                                     retreat_waypoints.waypoints.begin(), retreat_waypoints.waypoints.end());
    final_waypoints.waypoints.insert(final_waypoints.waypoints.end(),
                                     home_waypoints.waypoints.begin(), home_waypoints.waypoints.end());
    return final_waypoints;
}

// SLERP 헬퍼 함수 구현
tf2::Quaternion nrs_interpolation::quaternionSlerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, double t)
{
    double dot = q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w();
    tf2::Quaternion q2_copy = q2;
    if (dot < 0.0)
    {
        q2_copy = -q2_copy;
        dot = -dot;
    }
    if (dot > 0.9995)
    {
        tf2::Quaternion result = q1 * (1.0 - t) + q2_copy * t;
        result.normalize();
        return result;
    }
    double theta_0 = acos(dot);
    double theta = theta_0 * t;
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);
    double s1 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s2 = sin_theta / sin_theta_0;
    tf2::Quaternion result = (q1 * s1) + (q2_copy * s2);
    result.normalize();
    return result;
}

// 쿼터니언 기반 보간 함수 구현
nrs_path::Waypoints nrs_interpolation::interpolateXYZQF(const nrs_path::Waypoints &input, double desired_interval)
{
    nrs_path::Waypoints output;
    if (input.waypoints.empty())
        return output;

    std::vector<double> cumulative_distances;
    cumulative_distances.push_back(0.0);
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
    double total_distance = cumulative_distances.back();
    for (double d = 0.0; d <= total_distance; d += desired_interval)
    {
        size_t i = 1;
        while (i < cumulative_distances.size() && cumulative_distances[i] < d)
            i++;
        double t = (d - cumulative_distances[i - 1]) / (cumulative_distances[i] - cumulative_distances[i - 1]);
        const auto &p0 = input.waypoints[i - 1];
        const auto &p1 = input.waypoints[i];
        nrs_path::Waypoint interp_wp;
        interp_wp.x = p0.x + t * (p1.x - p0.x);
        interp_wp.y = p0.y + t * (p1.y - p0.y);
        interp_wp.z = p0.z + t * (p1.z - p0.z);
        tf2::Quaternion q0(p0.qx, p0.qy, p0.qz, p0.qw);
        tf2::Quaternion q1(p1.qx, p1.qy, p1.qz, p1.qw);
        tf2::Quaternion q_interp = quaternionSlerp(q0, q1, t);
        interp_wp.qw = q_interp.getW();
        interp_wp.qx = q_interp.getX();
        interp_wp.qy = q_interp.getY();
        interp_wp.qz = q_interp.getZ();
        interp_wp.Fx = p0.Fx + t * (p1.Fx - p0.Fx);
        interp_wp.Fy = p0.Fy + t * (p1.Fy - p0.Fy);
        interp_wp.Fz = p0.Fz + t * (p1.Fz - p0.Fz);
        output.waypoints.push_back(interp_wp);
    }
    return output;
}

nrs_path::Waypoints nrs_interpolation::interpolateEnd2End(const nrs_path::Waypoints &original_waypoints, double desired_interval,
                                                          const Triangle_mesh &mesh, double Fx, double Fy, double Fz)
{
    std::vector<geometry_msgs::Point> original_points;
    for (const auto &wp : original_waypoints.waypoints)
    {
        geometry_msgs::Point pt;
        pt.x = wp.x;
        pt.y = wp.y;
        pt.z = wp.z;
        original_points.push_back(pt);
    }

    std::vector<geometry_msgs::Point> approach_segment = generate_segment(original_points, 1, mesh);
    std::vector<geometry_msgs::Point> retreat_segment = generate_segment(original_points, 2, mesh);
    std::vector<geometry_msgs::Point> home_segment = generate_segment(original_points, 3, mesh);

    std::vector<geometry_msgs::Point> approach_interpolated = interpolatePoints(approach_segment, 0.001, 2);
    std::vector<geometry_msgs::Point> original_interpolated = interpolatePoints(original_points, 0.001, 1);
    std::vector<geometry_msgs::Point> retreat_interpolated = interpolatePoints(retreat_segment, 0.001, 2);
    std::vector<geometry_msgs::Point> home_interpolated = interpolatePoints(home_segment, 0.001, 2);

    nrs_path::Waypoints waypointsXYZQ = setToolVector(approach_interpolated, original_interpolated, retreat_interpolated, home_interpolated, mesh, Fx, Fy, Fz);
    nrs_path::Waypoints waypointsXYZQF = interpolateXYZQF(waypointsXYZQ, desired_interval);
    //return waypointsXYZQ;
    return waypointsXYZQF;
}

Eigen::Vector3d nrs_interpolation::getFaceNormal(const geometry_msgs::Point &ros_point, const Triangle_mesh &mesh)
{
    // ROS geometry_msgs::Point -> CGAL Point_3 변환
    Point_3 cgal_point(ros_point.x, ros_point.y, ros_point.z);

    // locate_face_and_point()를 사용하여 서피스에서 점과 면을 찾음
    face_descriptor face;
    CGAL::cpp11::array<double, 3> location; // Barycentric coordinates

    if (!n_geodesic.locate_face_and_point(cgal_point, face, location, mesh))
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