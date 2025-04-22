#include "nrs_geodesic.h"
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <limits>

// CGAL 관련 (locate_with_AABB_tree 등)
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

// 필요 시, 다른 보조 함수들 (예: locate_face_and_point)도 별도로 정의 가능

using namespace std;

void nrs_geodesic::geodesicbasecalcuation(const Eigen::Vector3d &p,
                                          const Eigen::Vector3d &q,
                                          Eigen::Vector3d &V_p,
                                          Eigen::Vector3d &V_q,
                                          double &geodesic_distance,
                                          const Triangle_mesh &tmesh,
                                          const std::vector<TriangleFace> &mesh)
{
    // 기존 path_generator.cpp의 geodesicbasecalcuation 내용 그대로 이동
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

    // 지오데식 거리 계산
    Surface_mesh_shortest_path shortest_paths(tmesh);
    shortest_paths.add_source_point(face2, location2);

    std::vector<Surface_mesh_shortest_path::Point_3> path_points;
    shortest_paths.shortest_path_points_to_source_points(face1, location1, std::back_inserter(path_points));

    auto result = shortest_paths.shortest_distance_to_source_points(face1, location1);
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
}

double nrs_geodesic::calculateAngleBetweenVectors(const Eigen::Vector3d &vec1,
                                                  const Eigen::Vector3d &vec2,
                                                  const Eigen::Vector3d &p,
                                                  const Triangle_mesh &tmesh)
{
    face_descriptor face_desc;
    Surface_mesh_shortest_path::Barycentric_coordinates bary_coords;

    Kernel::Point_3 cgal_p(p.x(), p.y(), p.z());
    if (!locate_face_and_point(cgal_p, face_desc, bary_coords, tmesh))
    {
        cerr << "Error: Failed to locate the face for the point." << endl;
        return 0.0;
    }

    // face_desc의 세 꼭지점
    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(face_desc), tmesh);
    auto v_it = vertices.begin();

    nrs_vec3d::Vec3d v1 = n_vec3d.cgalPointToVec3d(tmesh.point(*v_it++));
    nrs_vec3d::Vec3d v2 = n_vec3d.cgalPointToVec3d(tmesh.point(*v_it++));
    nrs_vec3d::Vec3d v3 = n_vec3d.cgalPointToVec3d(tmesh.point(*v_it));

    Eigen::Vector3d face_normal = n_vec3d.computeFaceNormal(v1, v2, v3, true);

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

Eigen::Vector3d nrs_geodesic::geodesicextend(const Eigen::Vector3d &p,
                                             const Eigen::Vector3d &q,
                                             const Eigen::Vector3d &V_q,
                                             const Triangle_mesh &tmesh,
                                             const std::vector<TriangleFace> &mesh,
                                             double angle)
{
    // face1, face2 찾기
    Kernel::Point_3 point1(p.x(), p.y(), p.z());
    Kernel::Point_3 point2(q.x(), q.y(), q.z());

    face_descriptor face1, face2;
    Surface_mesh_shortest_path::Barycentric_coordinates loc1, loc2;

    if (!locate_face_and_point(point1, face1, loc1, tmesh))
        throw std::runtime_error("Failed to locate point1 on mesh.");
    if (!locate_face_and_point(point2, face2, loc2, tmesh))
        throw std::runtime_error("Failed to locate point2 on mesh.");

    // face2의 노말 구하기
    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(face2), tmesh);
    auto v_it = vertices.begin();

    Kernel::Point_3 cp1 = tmesh.point(*v_it++);
    Kernel::Point_3 cp2 = tmesh.point(*v_it++);
    Kernel::Point_3 cp3 = tmesh.point(*v_it);

    nrs_vec3d::Vec3d v1 = n_vec3d.cgalPointToVec3d(cp1);
    nrs_vec3d::Vec3d v2 = n_vec3d.cgalPointToVec3d(cp2);
    nrs_vec3d::Vec3d v3 = n_vec3d.cgalPointToVec3d(cp3);

    Eigen::Vector3d normal = n_vec3d.computeFaceNormal(v1, v2, v3, true);

    if (normal.norm() == 0)
    {
        throw std::runtime_error("Invalid normal vector, cannot perform rotation.");
    }

    Eigen::Vector3d rotated_vector;

    Eigen::AngleAxisd rotation(-angle, normal.normalized());
    rotated_vector = rotation * V_q;

    return rotated_vector;
}

std::tuple<nrs_vec3d::Vec3d, nrs_vec3d::Vec3d>
nrs_geodesic::project_and_find_intersection(const nrs_vec3d::Vec3d &current_point,
                                            const nrs_vec3d::Vec3d &current_direction,
                                            double &distance_traveled,
                                            const Triangle_mesh &tmesh,
                                            const std::vector<TriangleFace> &mesh)
{
    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    nrs_vec3d::Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
    nrs_vec3d::Vec3d updated_point = current_point + epsilon_vector;

    locate_face_and_point(n_vec3d.vec3dToCgalPoint(updated_point), current_face_descriptor, barycentric_coords, tmesh);

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
    auto v_it = vertices.begin();

    Point_3 v1 = tmesh.point(*v_it++);
    Point_3 v2 = tmesh.point(*v_it++);
    Point_3 v3 = tmesh.point(*v_it);

    nrs_vec3d::Vec3d v1_vec = n_vec3d.cgalPointToVec3d(v1);
    nrs_vec3d::Vec3d v2_vec = n_vec3d.cgalPointToVec3d(v2);
    nrs_vec3d::Vec3d v3_vec = n_vec3d.cgalPointToVec3d(v3);

    nrs_vec3d::Vec3d projected_point = current_point;
    nrs_vec3d::Vec3d projected_direction = current_direction.normalize();

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

    nrs_vec3d::Vec3d final_point = v1_vec + (v2_vec - v1_vec) * bary_intersection.x() + (v3_vec - v1_vec) * bary_intersection.y();

    nrs_vec3d::Vec3d new_direction = (final_point - current_point).normalize();

    const double epsilon = 1e-6;

    if ((std::abs(final_point.x - current_point.x) < epsilon) &&
        (std::abs(final_point.y - current_point.y) < epsilon) &&
        (std::abs(final_point.z - current_point.z) < epsilon))
    {

        nrs_vec3d::Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
        nrs_vec3d::Vec3d offset_point = current_point + epsilon_vector;

        auto [updated_point, updated_direction] = project_and_find_intersection(offset_point, current_direction, distance_traveled, tmesh, mesh);

        return std::make_tuple(updated_point, updated_direction);
    }
    else
    {

        return std::make_tuple(final_point, new_direction);
    }
}

Eigen::Vector3d nrs_geodesic::geodesicAddVector(const Eigen::Vector3d &p,
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
    nrs_vec3d::Vec3d final_point;

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

    nrs_vec3d::Vec3d current_point = n_vec3d.eigenToVec3d(q);
    nrs_vec3d::Vec3d current_direction = n_vec3d.eigenToVec3d(start_direction).normalize();

    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    if (!locate_face_and_point(n_vec3d.vec3dToCgalPoint(current_point), current_face_descriptor, barycentric_coords, tmesh))
    {
        std::cerr << "Failed to locate point on mesh." << std::endl;
        return n_vec3d.vec3dToEigen(current_point);
    }

    while (true)
    {

        auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
        auto v_it = vertices.begin();

        Point_3 v1 = tmesh.point(*v_it++);
        Point_3 v2 = tmesh.point(*v_it++);
        Point_3 v3 = tmesh.point(*v_it);

        nrs_vec3d::Vec3d v1_vec = n_vec3d.cgalPointToVec3d(v1);
        nrs_vec3d::Vec3d v2_vec = n_vec3d.cgalPointToVec3d(v2);
        nrs_vec3d::Vec3d v3_vec = n_vec3d.cgalPointToVec3d(v3);

        Eigen::Vector3d current_normal = n_vec3d.computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        auto [new_point, new_direction] = project_and_find_intersection(current_point, current_direction, distance_traveled, tmesh, mesh);

        face_descriptor new_face_descriptor;

        locate_face_and_point(n_vec3d.vec3dToCgalPoint(new_point), new_face_descriptor, barycentric_coords, tmesh);

        if (new_face_descriptor == current_face_descriptor)
        {

            nrs_vec3d::Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
            nrs_vec3d::Vec3d updated_point = new_point + epsilon_vector;

            if (!locate_face_and_point(n_vec3d.vec3dToCgalPoint(updated_point), new_face_descriptor, barycentric_coords, tmesh))
            {
                std::cerr << "Failed to locate new point on mesh." << std::endl;
                return n_vec3d.vec3dToEigen(updated_point);
            }
        }

        vertices = CGAL::vertices_around_face(tmesh.halfedge(new_face_descriptor), tmesh);
        v_it = vertices.begin();

        v1 = tmesh.point(*v_it++);
        v2 = tmesh.point(*v_it++);
        v3 = tmesh.point(*v_it);

        v1_vec = n_vec3d.cgalPointToVec3d(v1);
        v2_vec = n_vec3d.cgalPointToVec3d(v2);
        v3_vec = n_vec3d.cgalPointToVec3d(v3);

        Eigen::Vector3d new_normal = n_vec3d.computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        new_direction = n_vec3d.rotateVectorToNewNormal(new_direction, current_normal, new_normal);

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

    return n_vec3d.vec3dToEigen(final_point);
}

double nrs_geodesic::computeGeodesicDistance(const Eigen::Vector3d &p0,
                                             const Eigen::Vector3d &p1,
                                             const Triangle_mesh &mesh)
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

std::vector<double>
nrs_geodesic::calculateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points,
                                               bool chord_length, const Triangle_mesh &tmesh)
{
    std::vector<double> u_values = {0.0};
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

// tranform triangle to basic calculation
std::vector<TriangleFace> nrs_geodesic::convertMeshToTriangleFaces(const Triangle_mesh &tmesh)
{
    std::vector<TriangleFace> triangle_faces;

    for (auto face : tmesh.faces())
    {
        TriangleFace triangle;

        int i = 0;
        for (auto vertex : vertices_around_face(tmesh.halfedge(face), tmesh))
        {
            Kernel::Point_3 p = tmesh.point(vertex);
            triangle.vertices[i] = n_vec3d.eigenToVec3d(Eigen::Vector3d(p.x(), p.y(), p.z()));
            i++;
        }

        triangle_faces.push_back(triangle);
    }

    return triangle_faces;
}

Eigen::Vector3d
nrs_geodesic::geodesicSubtract(const Eigen::Vector3d &p1,
                               const Eigen::Vector3d &p2,
                               const Triangle_mesh &tmesh)
{
    Eigen::Vector3d V_p, V_q;
    double geodesic_distance;
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    geodesicbasecalcuation(p1, p2, V_p, V_q, geodesic_distance, tmesh, mesh);

    return V_p * geodesic_distance;
}

std::vector<Eigen::Vector3d>
nrs_geodesic::calculateGeodesicTangentVectors(const std::vector<Eigen::Vector3d> &selected_points,
                                              const std::vector<double> &u_values,
                                              const Triangle_mesh &tmesh)
{
    float c = 0.2;

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

std::vector<std::vector<Eigen::Vector3d>>
nrs_geodesic::computeBezierControlPoints(const std::vector<Eigen::Vector3d> &selected_points,
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

    return bezier_control_points;
}

std::vector<Eigen::Vector3d>
nrs_geodesic::computeGeodesicBezierCurvePoints(const std::vector<Eigen::Vector3d> &control_points,
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

    Eigen::Vector3d q0;
    Eigen::Vector3d q1;
    Eigen::Vector3d q2;

    for (int i = 0; i < steps; ++i)
    {

        double t = static_cast<double>(i) / steps;

        q0 = geodesicAddVector(b0, v01.normalized(), 3 * (1 - t) * (1 - t) * t * v01.norm(), b0, tmesh, mesh);

        q1 = geodesicAddVector(b0, v02.normalized(), 3 * (1 - t) * t * t * v02.norm(), q0, tmesh, mesh);

        q2 = geodesicAddVector(b0, v03.normalized(), t * t * t * v03.norm(), q1, tmesh, mesh);

        curve_points.push_back(q2);
    }

    return curve_points;
}

bool nrs_geodesic::locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &tmesh)
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

nrs_path::Waypoints
nrs_geodesic::ConvertToWaypoints(const std::vector<geometry_msgs::Point> &points)
{
    // 기존 convertToWaypoints 내용을 이곳으로 이동
    nrs_path::Waypoints waypoints;

    for (const auto &point : points)
    {
        nrs_path::Waypoint waypoint_msg;
        waypoint_msg.x = point.x;
        waypoint_msg.y = point.y;
        waypoint_msg.z = point.z;
        // 자세, 법선, 쿼터니언 등을 계산하려면 추가 로직이 필요할 수 있음
        waypoint_msg.qw = 0;
        waypoint_msg.qx = 0;
        waypoint_msg.qy = 0;
        waypoint_msg.qz = 0;

        waypoints.waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}

nrs_path::Waypoints
nrs_geodesic::GenerateStraightGeodesicPath(const std::vector<Eigen::Vector3d> &points, const Triangle_mesh &tmesh)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Point_3> complete_path;
    std::vector<Point_3> path_segment;
    shortest_paths = new Surface_mesh_shortest_path(tmesh);
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

    nrs_path::Waypoints path_points;
    for (size_t i = 0; i < complete_path.size(); i++)
    {
        nrs_path::Waypoint wp;
        wp.x = complete_path[i].x();
        wp.y = complete_path[i].y();
        wp.z = complete_path[i].z();
        path_points.waypoints.push_back(wp);
    }

    ROS_INFO("Generated geodesic path with %zu points", path_points.waypoints.size());
    // 프로그램 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();

    // 시작과 종료 시간의 차이 계산 (밀리초 단위)
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    // 소요 시간 출력
    std::cout << "straight path generation time: " << duration << " s" << std::endl;
    return path_points;
}

nrs_path::Waypoints
nrs_geodesic::GenerateHermiteSplinePath(std::vector<Eigen::Vector3d> &points, const Triangle_mesh &tmesh)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector3d> hermite_spline;

    if (points.size() > 2)
    {

        std::vector<double> u_values = calculateInterpolationParameters(points, false, tmesh);


        std::vector<Eigen::Vector3d> tangent_vectors = calculateGeodesicTangentVectors(points, u_values, tmesh);

        std::vector<std::vector<Eigen::Vector3d>> bezier_control_points = computeBezierControlPoints(points, u_values, tangent_vectors, tmesh);

        int i = 0;
        for (const auto &control_points : bezier_control_points)
        {
            std::cout << "generating Spline bewteen point[" << i << "] and point[" << i + 1 << "]" << std::endl;
            double waypoints_distance = computeGeodesicDistance(points[i], points[i + 1], tmesh);
            int steps = waypoints_distance * 50;
            std::vector<Eigen::Vector3d> curve_points = computeGeodesicBezierCurvePoints(control_points, tmesh, steps);
            hermite_spline.insert(hermite_spline.end(), curve_points.begin(), curve_points.end());
            i += 1;
        }
    }
    nrs_path::Waypoints path_points;
    for (size_t i = 0; i < hermite_spline.size(); i++)
    {
        nrs_path::Waypoint wp;
        wp.x = hermite_spline[i].x();
        wp.y = hermite_spline[i].y();
        wp.z = hermite_spline[i].z();
        path_points.waypoints.push_back(wp);
    }

    ROS_INFO("Generated Hermite_Spline path with %zu points", path_points.waypoints.size());

    // 프로그램 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();

    // 시작과 종료 시간의 차이 계산 (밀리초 단위)
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    // 소요 시간 출력
    std::cout << "spline path generation time: " << duration << " s" << std::endl;
    return path_points;
}

bool nrs_geodesic::load_stl_file(std::ifstream &input, Triangle_mesh &mesh)
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