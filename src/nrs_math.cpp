#include "nrs_math.h"

#include <cmath>

void nrs_math::quaternionToRPY(double qx, double qy, double qz, double qw,
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
    roll = abs(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y축 회전)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::fabs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    // yaw (z축 회전)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // roll이 π보다 크면: π - (roll - π)
    if (roll > M_PI)
        roll = M_PI - (roll - M_PI);  // = 2*M_PI - roll 도 가능
}

