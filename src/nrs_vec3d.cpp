#include "nrs_vec3d.h"


// Eigen::Vector3d -> Vec3d 변환
nrs_vec3d::Vec3d nrs_vec3d::eigenToVec3d(const Eigen::Vector3d &eigen_vec) {
    return { eigen_vec.x(), eigen_vec.y(), eigen_vec.z() };
}

// Vec3d -> Eigen::Vector3d 변환
Eigen::Vector3d nrs_vec3d::vec3dToEigen(const Vec3d &v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

// CGAL::Point_3 -> Vec3d 변환
nrs_vec3d::Vec3d nrs_vec3d::cgalPointToVec3d(const Point_3 &p) {
    return { p.x(), p.y(), p.z() };
}

// Vec3d -> CGAL::Point_3 변환
Point_3 nrs_vec3d::vec3dToCgalPoint(const Vec3d &v) {
    return Point_3(v.x, v.y, v.z);
}

// 세 점으로부터 면의 법선 계산 (clockwise가 true이면 (v1->v2) x (v1->v3) 순)
Eigen::Vector3d nrs_vec3d::computeFaceNormal(const Vec3d &v1,
                                              const Vec3d &v2,
                                              const Vec3d &v3,
                                              bool clockwise)
{
    Eigen::Vector3d edge1 = vec3dToEigen(v2) - vec3dToEigen(v1);
    Eigen::Vector3d edge2 = vec3dToEigen(v3) - vec3dToEigen(v1);
    Eigen::Vector3d normal;
    if (clockwise)
        normal = edge1.cross(edge2);
    else
        normal = edge2.cross(edge1);
    return normal.normalized();
}

nrs_vec3d::Vec3d nrs_vec3d::rotateVectorToNewNormal(
    nrs_vec3d::Vec3d &vec,
    const Eigen::Vector3d &old_normal,
    const Eigen::Vector3d &new_normal)
{
    Eigen::Vector3d v = vec3dToEigen(vec);
    Eigen::Vector3d rotation_axis = old_normal.cross(new_normal);
    double angle = acos(old_normal.dot(new_normal) / (old_normal.norm() * new_normal.norm()));

    if (rotation_axis.norm() < 1e-6 || std::isnan(angle))
    {

        nrs_vec3d::Vec3d v2 = eigenToVec3d(v);
        return v2;
    }

    Eigen::AngleAxisd rotation(angle, rotation_axis.normalized());
    Eigen::Vector3d rotated_vec = rotation * v;
    rotated_vec = rotated_vec.normalized();
    nrs_vec3d::Vec3d rotated_vec2 = eigenToVec3d(rotated_vec);

    return rotated_vec2;
}