#ifndef NRS_MATH_H
#define NRS_MATH_H
#include <string>
#include <cmath>
#include <fstream>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>

// 네임스페이스 사용 (필요에 따라 수정)

using namespace std;

class nrs_math
{
public:

    void quaternionToRPY(double qx, double qy, double qz, double qw,
                         double &roll, double &pitch, double &yaw);
};

#endif // NRS_MATH_H
