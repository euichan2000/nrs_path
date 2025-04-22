#ifndef NRS_VEC3D_H
#define NRS_VEC3D_H

#include <cmath>
#include <Eigen/Dense>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

// CGAL Kernel 및 Point_3 정의
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;

/**
 * @brief nrs_vec3d 클래스
 * 벡터 연산 관련 함수들을 정적 멤버 함수로 제공하며,
 * Vec3d 구조체는 단순히 x, y, z 데이터만 보유합니다.
 */
class nrs_vec3d
{
public:
    // 단순 3차원 벡터 구조체
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

    // Eigen::Vector3d -> Vec3d 변환
    static Vec3d eigenToVec3d(const Eigen::Vector3d &eigen_vec);

    // Vec3d -> Eigen::Vector3d 변환
    static Eigen::Vector3d vec3dToEigen(const Vec3d &v);

    // CGAL::Point_3 -> Vec3d 변환
    static Vec3d cgalPointToVec3d(const Point_3 &p);

    // Vec3d -> CGAL::Point_3 변환
    static Point_3 vec3dToCgalPoint(const Vec3d &v);

    // 세 점으로부터 면의 법선 계산 (clockwise가 true이면 (v1->v2) x (v1->v3) 순)
    static Eigen::Vector3d computeFaceNormal(const Vec3d &v1,
                                             const Vec3d &v2,
                                             const Vec3d &v3,
                                             bool clockwise);

    static Vec3d rotateVectorToNewNormal(
        Vec3d &vec,
        const Eigen::Vector3d &old_normal,
        const Eigen::Vector3d &new_normal);
};

#endif // NRS_VEC3D_H
