#ifndef NRS_IO_H
#define NRS_IO_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/filesystem.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

#include <nrs_path/Waypoint.h>
#include <nrs_path/Waypoints.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>

#include "nrs_math.h"

namespace fs = boost::filesystem;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Point_set_3<Point_3> Point_set;

class nrs_io
{
public:
    nrs_math n_math;
    std::string file_path_;
    int file_counter = 0;

    // 파일 내용 지우기 함수
    void clearFile(const std::string &file_path);

    // Waypoints 파일 저장 함수 구현
    void saveWaypointsToFile(const nrs_path::Waypoints &final_waypoints,
                             const std::string &file_path);

    std::vector<Eigen::Vector3d> loadWaypointsFromFile(const std::string &file_path);

    nrs_path::Waypoints loadGeodesicWaypointsFromFile(const std::string &file_path);
    
    void sendFile(const std::string &file_path, ros::Publisher &file_pub);
};

#endif // NRS_IO_H
