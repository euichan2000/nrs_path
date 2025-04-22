#ifndef NRS_GEODESIC_H
#define NRS_GEODESIC_H

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <nrs_path/Waypoints.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL_reader.h>

// Vec3d 등 필요한 헤더
#include "nrs_vec3d.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

// 점과 노말을 저장하는 pair 및 리스트 정의
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;

// 메쉬 생성을 위한 컨테이너와 메쉬 타입 (여기서는 Surface_mesh 사용)
typedef std::vector<Point_3> Point_container;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Tree;
typedef boost::graph_traits<Triangle_mesh>::face_descriptor face_descriptor;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;

struct TriangleFace
{
    nrs_vec3d::Vec3d vertices[3];
    nrs_vec3d::Vec3d normal;
};

/**
 * @brief nrs_geodesic 클래스: 지오데식 계산, 스플라인/Bezier 관련 계산 등을 담당
 */
class nrs_geodesic
{
public:
    // CGAL 타입 정의
    Surface_mesh_shortest_path *shortest_paths;
    /**
     * @brief 메시의 한 삼각형(정점 3개, 노말) 정보를 담는 구조체
     */

    nrs_vec3d n_vec3d;

    /**
     * @brief P, Q 두 점에 대한 기본 지오데식 정보 계산
     * @param p, q   : 입력 점 (Eigen::Vector3d)
     * @param V_p, V_q : 결과 벡터 (출발점, 도착점 근방의 방향)
     * @param geodesic_distance : P->Q까지의 지오데식 거리
     * @param tmesh  : 사용 중인 메시
     * @param mesh   : TriangleFace 배열(노말 계산 등에 사용)
     */
    void geodesicbasecalcuation(const Eigen::Vector3d &p,
                                const Eigen::Vector3d &q,
                                Eigen::Vector3d &V_p,
                                Eigen::Vector3d &V_q,
                                double &geodesic_distance,
                                const Triangle_mesh &tmesh,
                                const std::vector<TriangleFace> &mesh);

    /**
     * @brief 두 벡터 사이의 각도 계산
     */
    double calculateAngleBetweenVectors(const Eigen::Vector3d &vec1,
                                        const Eigen::Vector3d &vec2,
                                        const Eigen::Vector3d &p,
                                        const Triangle_mesh &tmesh);

    /**
     * @brief P->Q 지오데식에서 V_p와 동일한 각도를 갖는 V_q 벡터 계산
     */
    Eigen::Vector3d geodesicextend(const Eigen::Vector3d &p,
                                   const Eigen::Vector3d &q,
                                   const Eigen::Vector3d &V_q,
                                   const Triangle_mesh &tmesh,
                                   const std::vector<TriangleFace> &mesh,
                                   double angle);

    /**
     * @brief 현재 점과 방향 벡터를 메시에 투영 후, 다음 엣지 교차점 찾기
     * @return (최종 점, 새 방향 벡터)의 튜플
     */
    std::tuple<nrs_vec3d::Vec3d, nrs_vec3d::Vec3d> project_and_find_intersection(const nrs_vec3d::Vec3d &current_point,
                                                                                 const nrs_vec3d::Vec3d &current_direction,
                                                                                 double &distance_traveled,
                                                                                 const Triangle_mesh &tmesh,
                                                                                 const std::vector<TriangleFace> &mesh);

    /**
     * @brief 메시 상에서 p->q 지오데식 근방 방향을 start_direction_p로 확장하여 total_distance만큼 이동
     * @return 최종 이동 후의 3D 좌표
     */
    Eigen::Vector3d geodesicAddVector(const Eigen::Vector3d &p,
                                      const Eigen::Vector3d &start_direction_p,
                                      double total_distance,
                                      const Eigen::Vector3d &q,
                                      const Triangle_mesh &tmesh,
                                      const std::vector<TriangleFace> &mesh);

    /**
     * @brief 메시 상에서 두 점 사이의 지오데식 거리 계산
     */
    double computeGeodesicDistance(const Eigen::Vector3d &p0,
                                   const Eigen::Vector3d &p1,
                                   const Triangle_mesh &mesh);

    /**
     * @brief 선택된 지점들에 대한 보간 파라미터 계산 (chord_length 등)
     */
    std::vector<double> calculateInterpolationParameters(std::vector<Eigen::Vector3d> &selected_points,
                                                         bool chord_length, const Triangle_mesh &tmesh);

    std::vector<TriangleFace> convertMeshToTriangleFaces(const Triangle_mesh &tmesh);

    /**
     * @brief 지오데식 도메인에서 두 점의 차(접선 방향) 계산
     */
    Eigen::Vector3d geodesicSubtract(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2,
                                     const Triangle_mesh &tmesh);

    /**
     * @brief 지오데식 탄젠트 벡터 계산
     */
    std::vector<Eigen::Vector3d> calculateGeodesicTangentVectors(const std::vector<Eigen::Vector3d> &selected_points,
                                                                 const std::vector<double> &u_values,
                                                                 const Triangle_mesh &tmesh);

    /**
     * @brief Bézier 스플라인 제어점 계산
     */
    std::vector<std::vector<Eigen::Vector3d>> computeBezierControlPoints(const std::vector<Eigen::Vector3d> &selected_points,
                                                                         const std::vector<double> &u_values,
                                                                         const std::vector<Eigen::Vector3d> &tangent_vectors,
                                                                         const Triangle_mesh &tmesh);

    /**
     * @brief Bézier 스플라인으로부터 실제 곡선상의 점들 계산
     */
    std::vector<Eigen::Vector3d> computeGeodesicBezierCurvePoints(const std::vector<Eigen::Vector3d> &control_points,
                                                                  const Triangle_mesh &tmesh,
                                                                  int steps);
    bool locate_face_and_point(const Kernel::Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location, const Triangle_mesh &tmesh);

    nrs_path::Waypoints
    ConvertToWaypoints(const std::vector<geometry_msgs::Point> &points);

    /**
     * @brief 주어진 점들로부터 지오데식(최단거리) 경로를 생성
     * @param points 경로를 구성하는 Eigen::Vector3d 배열
     * @param tmesh  사용 중인 Triangle_mesh
     */
    nrs_path::Waypoints
    GenerateStraightGeodesicPath(const std::vector<Eigen::Vector3d> &points, const Triangle_mesh &tmesh);

    /**
     * @brief 주어진 점들로부터 Hermite Spline을 이용해 경로를 생성
     * @param points 경로를 구성하는 Eigen::Vector3d 배열
     * @param tmesh           사용 중인 Triangle_mesh
     */
    nrs_path::Waypoints
    GenerateHermiteSplinePath(std::vector<Eigen::Vector3d> &points, const Triangle_mesh &tmesh);

    bool load_stl_file(std::ifstream &input, Triangle_mesh &mesh);
};

#endif // NRS_GEODESIC_H
