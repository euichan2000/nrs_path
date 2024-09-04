#include "visualization.h"




class path
{
public:
void generate_Geodesic_Path(const std::vector<geometry_msgs::Point> &points);
void generate_B_Spline_Path(const std::vector<geometry_msgs::Point> &points);
void generate_Catmull_Rom_Path(const std::vector<geometry_msgs::Point> &points);
void generate_Hermite_Spline_path(const std::vector<std::vector<Eigen::Vector3d>> &bezier_control_points,int steps);
};