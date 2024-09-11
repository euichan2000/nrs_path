#include "path.h"

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

Triangle_mesh mesh;
Tree *tree;
Surface_mesh_shortest_path *shortest_paths;

struct Vec3d
{
    double x, y, z;

    Vec3d() : x(0), y(0), z(0) {}

    Vec3d(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3d operator-(const Vec3d &other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3d operator+(const Vec3d &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3d operator*(double scalar) const
    {
        return {x * scalar, y * scalar, z * scalar};
    }
    // double * Vec3d
    Vec3d operator*(double scalar, const Vec3d &vec)
    {
        return {scalar * vec.x, scalar * vec.y, scalar * vec.z};
    }

    Vec3d cross(const Vec3d &other) const
    {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x};
    }

    double dot(const Vec3d &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    double length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3d normalize() const
    {
        double len = length();
        if (len > 0)
        {
            return {x / len, y / len, z / len};
        }
        return {0, 0, 0};
    }
};

// Triangle bertices & Normal
struct TriangleFace
{
    Vec3d vertices[3];
    Vec3d normal;
    const TriangleFace *neighbors[3];
};

class cgal_related
{
    bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh);
    std::vector<nrs_path_planning::Waypoint> convertToWaypoints(const std::vector<geometry_msgs::Point> &points, const Triangle_mesh mesh);

    bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &mesh);
    std::vector<geometry_msgs::Point> projectPointsOntoMesh(const std::vector<geometry_msgs::Point> &points);

    Vec3d EigenToVec3d(const Eigen::Vector3d &eigen_vec);
    Eigen::Vector3d Vec3dToEigen(const Vec3d &vec);
    Vec3d CGALPointToVec3d(const Point_3 &p);
    Point_3 Vec3dToCGALPoint(const Vec3d &v);
    std::vector<TriangleFace> convertMeshToTriangleFaces(const Triangle_mesh &mesh);

    void geodesicbasecalcuation(const Eigen::Vector3d &p, const Eigen::Vector3d &q, Eigen::Vector3d &V_p, Eigen::Vector3d &V_q, double &geodesic_distance, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh);
    double computeGeodesicDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Triangle_mesh &mesh);
    Eigen::Vector3d geodesicSubtract(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array, int &marker_id);
    Eigen::Vector3d computeFaceNormal(const Vec3d &v1, const Vec3d &v2, const Vec3d &v3, bool clockwise = true);

    Vec3d rotateVectorToNewNormal(Vec3d &vec, const Eigen::Vector3d &old_normal, const Eigen::Vector3d &new_normal);
    double calculateAngleBetweenVectors(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &p, const Triangle_mesh &tmesh);
    Eigen::Vector3d geodesicextend(const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &V_q, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh, double angle);
    std::tuple<Vec3d, Vec3d> project_and_find_intersection(const Vec3d &current_point, const Vec3d &current_direction, double &distance_traveled, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh);
    Eigen::Vector3d geodesicAddVector(const Eigen::Vector3d &p, const Eigen::Vector3d &start_direction_p, double total_distance, const Eigen::Vector3d &q, const Triangle_mesh &tmesh, const std::vector<TriangleFace> &mesh);
    void updateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points, bool chord_length = true);
    std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(const std::vector<Eigen::Vector3d> &selected_points, const std::vector<double> &u_values, const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array, int &marker_id);
    std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(const std::vector<Eigen::Vector3d> &selected_points, const std::vector<double> &u_values, const std::vector<Eigen::Vector3d> &tangent_vectors, const Triangle_mesh &tmesh, visualization_msgs::MarkerArray &marker_array);
    std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(const std::vector<Eigen::Vector3d> &control_points, const Triangle_mesh &tmesh, int steps, visualization_msgs::MarkerArray &marker_array, int &marker_id);
};