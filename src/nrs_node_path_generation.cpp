#include "nrs_callback.h"
#include "nrs_visualization.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <iostream>

// CGAL 및 nrs_mesh 관련 헤더는 nrs_callback.h 에서 이미 include 되었다고 가정

// 전역 변수들
nrs_callback callback_handler;
std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_path/mesh/workpiece.stl";
Triangle_mesh tmesh;

// [생성] : "/clicked_point" 토픽 콜백 함수 (경로 생성)
void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    std::cout << "-----------------------------------new waypoints coming---------------------------------" << std::endl;

    Kernel::Point_3 clicked_point(msg->point.x, msg->point.y, msg->point.z);

    face_descriptor face;
    Surface_mesh_shortest_path::Barycentric_coordinates location;

    if (!callback_handler.n_geodesic.locate_face_and_point(clicked_point, face, location, tmesh))
    {
        std::cerr << "Failed to locate the clicked point on the mesh." << std::endl;
        return;
    }

    Kernel::Point_3 v1, v2, v3;
    int i = 0;
    for (auto v : vertices_around_face(tmesh.halfedge(face), tmesh))
    {
        if (i == 0)
            v1 = tmesh.point(v);
        else if (i == 1)
            v2 = tmesh.point(v);
        else if (i == 2)
            v3 = tmesh.point(v);
        ++i;
    }

    Kernel::Point_3 projected_point = CGAL::barycenter(v1, location[0], v2, location[1], v3, location[2]);

    Eigen::Vector3d projected_point_eigen(projected_point.x(), projected_point.y(), projected_point.z());

    callback_handler.selected_points.push_back(projected_point_eigen);
}

// [보간] : "/geodesic_path" 토픽 콜백 함수 (경로 보간)
void geodesicPathCallback(const nrs_path::Waypoints::ConstPtr &msg)
{
    for (const auto &wp : msg->waypoints)
    {
        callback_handler.geodesic_path.waypoints.push_back(wp);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_combined_node");
    ros::NodeHandle nh;
    nrs_visualization visualizer;
    visualizer.init(nh);

    // ===== 경로 생성 (Path Generation) 설정 =====
    callback_handler.mesh_file_path = mesh_file_path;
    callback_handler.geodesic_waypoints_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/geodesic_waypoints.txt";
    callback_handler.geodesic_waypoints_pub = nh.advertise<nrs_path::Waypoints>("geodesic_path", 10);

    // ===== 경로 보간 (Path Interpolation) 설정 =====
    callback_handler.interpolated_waypoints_file_path = "/home/nrs/catkin_ws/src/nrs_path/data/final_waypoints.txt";
    callback_handler.desired_interval = 0.00004;
    callback_handler.Fx = 0.0;
    callback_handler.Fy = 0.0;
    callback_handler.Fz = 10.0;
    callback_handler.interpolated_waypoints_pub = nh.advertise<nrs_path::Waypoints>("interpolated_waypoints", 10);

    // mesh 파일 로드
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input)
    {
        ROS_ERROR("Failed to open mesh file: %s", mesh_file_path.c_str());
        return -1;
    }
    callback_handler.n_geodesic.load_stl_file(input, tmesh);

    // 두 서비스 서버 등록 (spline, straight 경로 생성 서비스)
    ros::ServiceServer spline_service = nh.advertiseService("spline",
                                                            &nrs_callback::splinePathServiceCallback,
                                                            &callback_handler);
    ros::ServiceServer straight_service = nh.advertiseService("straight",
                                                              &nrs_callback::straightPathServiceCallback,
                                                              &callback_handler);
    // 보간 서비스 서버 등록
    ros::ServiceServer interpolation_service = nh.advertiseService("interpolate",
                                                                   &nrs_callback::PathInterpolationCallback,
                                                                   &callback_handler);
    // 경로 삭제를 위한 서비스 서버 등록 (pathDeleteCallback)
    ros::ServiceServer path_delete_service = nh.advertiseService("delete",
                                                                 &nrs_callback::pathDeleteCallback,
                                                                 &callback_handler);

    // 클릭된 포인트 수신 구독자 (경로 생성용)
    ros::Subscriber clicked_point_sub = nh.subscribe("/clicked_point", 1000, clickedPointCallback);
    // "geodesic_path" 토픽 구독 (보간용)
    ros::Subscriber geodesic_path_sub = nh.subscribe("/geodesic_path", 1000, geodesicPathCallback);
    // ===== 경로 시각화 (Visualization) 설정 =====
    ros::Subscriber vis_waypoints_sub = nh.subscribe("interpolated_waypoints", 10, &nrs_visualization::waypointsCallback, &visualizer);
    
    ros::Subscriber vis_clicked_point_sub = nh.subscribe("/clicked_point", 10, &nrs_visualization::visualizeClickedPoint, &visualizer);


    ROS_INFO("Combined nrs node started. Generation, interpolation and visualization functionalities are active.");

    ros::spin();
    return 0;
}
