#include "cgal_related.h"

bool cgal_related::read_stl_file(std::ifstream &input, Triangle_mesh &mesh)
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
std::vector<nrs_vision_rviz::Waypoint> cgal_related::convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh mesh)
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

        waypoints.push_back(waypoint_msg);
    }

    return waypoints;
}
bool cgal_related::locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &mesh)
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
std::vector<geometry_msgs::Point> cgal_related::projectPointsOntoMesh(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<geometry_msgs::Point> projected_points, offset_points;

    for (const auto &point : points)
    {

        Point_3 query(point.x, point.y, point.z + 0.05);
        auto closest_point = tree->closest_point(query);

        geometry_msgs::Point off_point;
        off_point.x = query.x();
        off_point.y = query.y();
        off_point.z = query.z();
        offset_points.push_back(off_point);

        geometry_msgs::Point ros_point;
        ros_point.x = closest_point.x();
        ros_point.y = closest_point.y();
        ros_point.z = closest_point.z();

        projected_points.push_back(ros_point);
    }

    visualizePath(offset_points, "offset_B_spline_path", 1, 0.0, 0.0, 1.0, 1.0);
    visualizePath(projected_points, "projected_B_spline_path", 1, 1.0, 0.0, 0.0, 1.0);

    return projected_points;
}

Vec3d cgal_related::EigenToVec3d(const Eigen::Vector3d &eigen_vec)
{
    return {eigen_vec.x(), eigen_vec.y(), eigen_vec.z()};
}

Eigen::Vector3d cgal_related::Vec3dToEigen(const Vec3d &vec)
{
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

Vec3d cgal_related::CGALPointToVec3d(const Point_3 &p)
{
    return Vec3d(p.x(), p.y(), p.z());
}

Point_3 cgal_related::Vec3dToCGALPoint(const Vec3d &v)
{
    return Point_3(v.x, v.y, v.z);
}

std::vector<TriangleFace> cgal_related::convertMeshToTriangleFaces(const Triangle_mesh &mesh)
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
void cgal_related::geodesicbasecalcuation(const Eigen::Vector3d &p, const Eigen::Vector3d &q, Eigen::Vector3d &V_p, Eigen::Vector3d &V_q, double &geodesic_distance, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh)
{
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

    if (face1 != face2)
    {

        Surface_mesh_shortest_path shortest_paths(tmesh);
        shortest_paths.add_source_point(face2, location2);

        std::vector<Surface_mesh_shortest_path::Point_3> path_points;
        shortest_paths.shortest_path_points_to_source_points(face1, location1, std::back_inserter(path_points));

        std::pair<double, Surface_mesh_shortest_path::Source_point_iterator> result = shortest_paths.shortest_distance_to_source_points(face1, location1);
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
    else
    {

        V_p = q - p;
        V_q = q - p;
        V_p.normalize();
        V_q.normalize();
        geodesic_distance = V_p.norm();
    }
}
double cgal_related::computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh)
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
Eigen::Vector3d cgal_related::geodesicSubtract(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array, int &marker_id)
{
    Eigen::Vector3d V_p, V_q;
    double geodesic_distance;
    std::vector<TriangleFace> mesh = convertMeshToTriangleFaces(tmesh);
    geodesicbasecalcuation(p1, p2, V_p, V_q, geodesic_distance, tmesh, mesh);

    return V_p * geodesic_distance;
}
Eigen::Vector3d cgal_related::computeFaceNormal(const Vec3d &v1, const Vec3d &v2, const Vec3d &v3, bool clockwise = true)
{
    Eigen::Vector3d edge1 = Vec3dToEigen(v2) - Vec3dToEigen(v1);
    Eigen::Vector3d edge2 = Vec3dToEigen(v3) - Vec3dToEigen(v1);

    Eigen::Vector3d normal;
    if (clockwise)
    {
        normal = edge1.cross(edge2);
    }
    else
    {
        normal = edge2.cross(edge1);
    }

    return normal.normalized();
}

Vec3d cgal_related::rotateVectorToNewNormal(Vec3d &vec, const Eigen::Vector3d &old_normal, const Eigen::Vector3d &new_normal)
{
    Eigen::Vector3d v = Vec3dToEigen(vec);
    Eigen::Vector3d rotation_axis = old_normal.cross(new_normal);
    double angle = acos(old_normal.dot(new_normal) / (old_normal.norm() * new_normal.norm()));

    if (rotation_axis.norm() < 1e-6 || std::isnan(angle))
    {

        Vec3d v2 = EigenToVec3d(v);
        return v2;
    }

    Eigen::AngleAxisd rotation(angle, rotation_axis.normalized());
    Eigen::Vector3d rotated_vec = rotation * v;
    rotated_vec = rotated_vec.normalized();
    Vec3d rotated_vec2 = EigenToVec3d(rotated_vec);

    return rotated_vec2;
}
double cgal_related::calculateAngleBetweenVectors(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &p, const Triangle_mesh &tmesh)
{

    face_descriptor face_desc;
    Surface_mesh_shortest_path::Barycentric_coordinates bary_coords;

    if (!locate_face_and_point(Vec3dToCGALPoint(Vec3d(p.x(), p.y(), p.z())), face_desc, bary_coords, tmesh))
    {
        std::cerr << "Error: Failed to locate the face for the point." << std::endl;
        return 0.0;
    }

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(face_desc), tmesh);
    auto v_it = vertices.begin();

    Vec3d v1 = CGALPointToVec3d(tmesh.point(*v_it++));
    Vec3d v2 = CGALPointToVec3d(tmesh.point(*v_it++));
    Vec3d v3 = CGALPointToVec3d(tmesh.point(*v_it));

    Eigen::Vector3d face_normal = computeFaceNormal(v1, v2, v3);

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
Eigen::Vector3d cgal_related::geodesicextend(const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &V_q, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh, double angle)
{
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

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(face2), tmesh);
    auto v_it = vertices.begin();

    Point_3 v1 = tmesh.point(*v_it++);
    Point_3 v2 = tmesh.point(*v_it++);
    Point_3 v3 = tmesh.point(*v_it);

    Vec3d v1_vec = CGALPointToVec3d(v1);
    Vec3d v2_vec = CGALPointToVec3d(v2);
    Vec3d v3_vec = CGALPointToVec3d(v3);

    Eigen::Vector3d normal = computeFaceNormal(v1_vec, v2_vec, v3_vec);

    if (normal.norm() == 0)
    {
        throw std::runtime_error("Invalid normal vector, cannot perform rotation.");
    }

    Eigen::Vector3d rotated_vector;

    Eigen::AngleAxisd rotation(-angle, normal.normalized());
    rotated_vector = rotation * V_q;

    return rotated_vector;
}
std::tuple<Vec3d, Vec3d> cgal_related::project_and_find_intersection(const Vec3d &current_point, const Vec3d &current_direction, double &distance_traveled, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh)
{
    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
    Vec3d updated_point = current_point + epsilon_vector;

    locate_face_and_point(Vec3dToCGALPoint(updated_point), current_face_descriptor, barycentric_coords, tmesh);

    auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
    auto v_it = vertices.begin();

    Point_3 v1 = tmesh.point(*v_it++);
    Point_3 v2 = tmesh.point(*v_it++);
    Point_3 v3 = tmesh.point(*v_it);

    Vec3d v1_vec = CGALPointToVec3d(v1);
    Vec3d v2_vec = CGALPointToVec3d(v2);
    Vec3d v3_vec = CGALPointToVec3d(v3);

    Vec3d projected_point = current_point;
    Vec3d projected_direction = current_direction.normalize();

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

    Vec3d final_point = v1_vec + bary_intersection.x() * (v2_vec - v1_vec) + bary_intersection.y() * (v3_vec - v1_vec);

    Vec3d new_direction = (final_point - current_point).normalize();

    const double epsilon = 1e-6;

    if ((std::abs(final_point.x - current_point.x) < epsilon) &&
        (std::abs(final_point.y - current_point.y) < epsilon) &&
        (std::abs(final_point.z - current_point.z) < epsilon))
    {

        Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
        Vec3d offset_point = current_point + epsilon_vector;

        auto [updated_point, updated_direction] = project_and_find_intersection(offset_point, current_direction, distance_traveled, tmesh, mesh);

        return std::make_tuple(updated_point, updated_direction);
    }
    else
    {

        return std::make_tuple(final_point, new_direction);
    }
}
Eigen::Vector3d cgal_related::geodesicAddVector(const Eigen::Vector3d &p, const Eigen::Vector3d &start_direction_p, double total_distance, const Eigen::Vector3d &q, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh)
{

    Eigen::Vector3d V_p;
    Eigen::Vector3d V_q;
    double geodesic_distance;
    double distance_traveled = 0.0;
    Eigen::Vector3d start_direction;
    double percentage_traveled;
    Vec3d final_point;

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

    Vec3d current_point = EigenToVec3d(q);
    Vec3d current_direction = EigenToVec3d(start_direction).normalize();

    face_descriptor current_face_descriptor;
    Surface_mesh_shortest_path::Barycentric_coordinates barycentric_coords;

    if (!locate_face_and_point(Vec3dToCGALPoint(current_point), current_face_descriptor, barycentric_coords, tmesh))
    {
        std::cerr << "Failed to locate point on mesh." << std::endl;
        return Vec3dToEigen(current_point);
    }

    while (true)
    {

        auto vertices = CGAL::vertices_around_face(tmesh.halfedge(current_face_descriptor), tmesh);
        auto v_it = vertices.begin();

        Point_3 v1 = tmesh.point(*v_it++);
        Point_3 v2 = tmesh.point(*v_it++);
        Point_3 v3 = tmesh.point(*v_it);

        Vec3d v1_vec = CGALPointToVec3d(v1);
        Vec3d v2_vec = CGALPointToVec3d(v2);
        Vec3d v3_vec = CGALPointToVec3d(v3);

        Eigen::Vector3d current_normal = computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        auto [new_point, new_direction] = project_and_find_intersection(current_point, current_direction, distance_traveled, tmesh, mesh);

        face_descriptor new_face_descriptor;

        locate_face_and_point(Vec3dToCGALPoint(new_point), new_face_descriptor, barycentric_coords, tmesh);

        if (new_face_descriptor == current_face_descriptor)
        {

            Vec3d epsilon_vector = current_direction.normalize() * 1e-4;
            Vec3d updated_point = new_point + epsilon_vector;

            if (!locate_face_and_point(Vec3dToCGALPoint(updated_point), new_face_descriptor, barycentric_coords, tmesh))
            {
                std::cerr << "Failed to locate new point on mesh." << std::endl;
                return Vec3dToEigen(updated_point);
            }
        }

        vertices = CGAL::vertices_around_face(tmesh.halfedge(new_face_descriptor), tmesh);
        v_it = vertices.begin();

        v1 = tmesh.point(*v_it++);
        v2 = tmesh.point(*v_it++);
        v3 = tmesh.point(*v_it);

        v1_vec = CGALPointToVec3d(v1);
        v2_vec = CGALPointToVec3d(v2);
        v3_vec = CGALPointToVec3d(v3);

        Eigen::Vector3d new_normal = computeFaceNormal(v1_vec, v2_vec, v3_vec, true);

        new_direction = rotateVectorToNewNormal(new_direction, current_normal, new_normal);

        double new_distance_traveled = sqrt((new_point.x - current_point.x) * (new_point.x - current_point.x) + (new_point.y - current_point.y) * (new_point.y - current_point.y) + (new_point.z - current_point.z) * (new_point.z - current_point.z));

        if (distance_traveled + new_distance_traveled >= abs(total_distance))
        {
            double remaining_distance = abs(total_distance) - distance_traveled;
            final_point = current_point + current_direction.normalize() * remaining_distance;

            distance_traveled += remaining_distance;
            percentage_traveled = (distance_traveled / abs(total_distance)) * 100.0;

            break;
        }

        current_face_descriptor = new_face_descriptor;
        current_point = new_point;
        current_direction = new_direction;
        distance_traveled += new_distance_traveled;

        percentage_traveled = (distance_traveled / abs(total_distance)) * 100.0;
    }

    return Vec3dToEigen(final_point);
}

void cgal_related::updateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points, bool chord_length = true)
{
    for (int i = 1; i < selected_points.size(); i++)
    {
        double geodesic_distance = computeGeodesicDistance(selected_points[i], selected_points[i + 1], mesh);

        if (!chord_length)
        {
            geodesic_distance = std::sqrt(geodesic_distance);
        }

        double ui = u_values.back() + geodesic_distance;
        u_values.push_back(ui);
    }
}
std::vector<Eigen::Vector3d> cgal_related::calculateGeodesicTangentVectors(const std::vector<Eigen::Vector3d> &selected_points, const std::vector<double> &u_values, const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array, int &marker_id)
{
    int c = 0;

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

        Eigen::Vector3d tangent_vector = (1 - c) * (geodesicSubtract(p_prev, p_next, tmesh, marker_array, marker_id)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));

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

        Eigen::Vector3d tangent_vector = (1 - c) * (geodesicSubtract(p_prev, p_next, tmesh, marker_array, marker_id)) / (u_values[i + 1] - u_values[i - 1]);
        tangent_vectors.push_back(tangent_vector);
    }

    if (p_first == p_last)
    {

        Eigen::Vector3d p_prev = selected_points[1];
        Eigen::Vector3d p_now = selected_points[0];
        Eigen::Vector3d p_next = selected_points[selected_points.size() - 2];
        Eigen::Vector3d tangent_vector = (1 - c) * (geodesicSubtract(p_prev, p_next, tmesh, marker_array, marker_id)) / ((u_values[1] - u_values[0]) + (u_values.back() - u_values[u_values.size() - 2]));
        tangent_vectors.push_back(tangent_vector);
    }
    else
    {

        tangent_vectors.push_back(Eigen::Vector3d::Zero());
    }

    return tangent_vectors;
}
std::vector<std::vector<Eigen::Vector3d>> cgal_related::computeBezierControlPoints(const std::vector<Eigen::Vector3d> &selected_points, const std::vector<double> &u_values, const std::vector<Eigen::Vector3d> &tangent_vectors, const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array)
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

    for (int i = 0; i < bezier_control_points.size(); i++)
    {

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
std::vector<Eigen::Vector3d> cgal_related::computeGeodesicBezierCurvePoints(const std::vector<Eigen::Vector3d> &control_points, const Triangle_mesh &tmesh, int steps, visualization_msgs::MarkerArray &marker_array, int &marker_id)
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

    Eigen::Vector3d v01 = geodesicSubtract(b0, b1, tmesh, marker_array, marker_id);
    Eigen::Vector3d v02 = geodesicSubtract(b0, b2, tmesh, marker_array, marker_id);
    Eigen::Vector3d v03 = geodesicSubtract(b0, b3, tmesh, marker_array, marker_id);
    for (int i = 0; i < steps; ++i)
    {

        double t = static_cast<double>(i) / steps;

        Eigen::Vector3d q0 = geodesicAddVector(b0, v01, 3 * (1 - t) * (1 - t) * t * v01.norm(), b0, tmesh, mesh);

        Eigen::Vector3d q1 = geodesicAddVector(b0, v02, 3 * (1 - t) * t * t * v02.norm(), q0, tmesh, mesh);

        Eigen::Vector3d q2 = geodesicAddVector(b0, v03, t * t * t * v03.norm(), q1, tmesh, mesh);

        curve_points.push_back(q2);
        std::cout << "curve_points[" << i << "]: " << curve_points[i].x() << "|" << curve_points[i].y() << "|" << curve_points[i].z() << std::endl;
    }

    return curve_points;
}