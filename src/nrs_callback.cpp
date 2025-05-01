#include "nrs_callback.h"

nrs_callback::nrs_callback()
{
}

bool nrs_callback::splinePathServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Spline path service called. Generating Hermite Spline Path...");
    std::ifstream input(mesh_file_path, std::ios::binary);
    Triangle_mesh tmesh;
    n_geodesic.load_stl_file(input, tmesh);
    Tree *tree;
    Surface_mesh_shortest_path *shortest_paths;
    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(tmesh);
    // 웨이포인트 메시지 초기화
    waypoints_msg.waypoints.clear();
    selected_points = n_io.loadWaypointsFromFile(selected_waypoints_file_path);
    // 최소 3개 이상의 선택된 점이 필요
    if (selected_points.size() > 2)
    {
        // Hermite Spline 경로 생성 (selected_points와 tmesh 사용)
        nrs_path::Waypoints path_points = n_geodesic.GenerateHermiteSplinePath(selected_points, tmesh);

        // 생성된 웨이포인트 퍼블리시
        geodesic_waypoints_pub.publish(path_points);
        ROS_INFO("Published Hermite Spline Path with %zu waypoints", path_points.waypoints.size());

        n_io.clearFile(geodesic_waypoints_file_path);
        n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
        return true;
    }
    else
    {
        ROS_WARN("Not enough selected points for Hermite Spline Path generation.");
        return false;
    }
}

// Geodesic (직선) 경로 생성 서비스 콜백 함수
bool nrs_callback::straightPathServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Straight path service called. Generating Geodesic Path...");
    std::ifstream input(mesh_file_path, std::ios::binary);
    Triangle_mesh tmesh;
    n_geodesic.load_stl_file(input, tmesh);
    Tree *tree;
    Surface_mesh_shortest_path *shortest_paths;
    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(tmesh);
    waypoints_msg.waypoints.clear();
    selected_points = n_io.loadWaypointsFromFile(selected_waypoints_file_path);
    // 최소 2개 이상의 선택된 점이 필요
    if (selected_points.size() > 1)
    {
        // Geodesic 경로 생성 (selected_points와 tmesh 사용)
        nrs_path::Waypoints path_points = n_geodesic.GenerateStraightGeodesicPath(selected_points, tmesh);

        // 생성된 웨이포인트 퍼블리시
        geodesic_waypoints_pub.publish(path_points);

        ROS_INFO("Published Geodesic Path with %zu waypoints", path_points.waypoints.size());
        n_io.clearFile(geodesic_waypoints_file_path);
        n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
        return true;
    }
    else
    {
        ROS_WARN("Not enough selected points for Geodesic Path generation.");
        return false;
    }
}

// Waypoints 콜백 함수 구현
bool nrs_callback::PathInterpolationCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // 1. STL 파일 열기
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input.is_open())
    {
        ROS_ERROR("Failed to open mesh file: %s", mesh_file_path.c_str());
        return false;
    }

    // 2. STL 파일에서 mesh 로드
    Triangle_mesh tmesh;
    if (!n_geodesic.load_stl_file(input, tmesh))
    {
        ROS_ERROR("Failed to load mesh from file: %s", mesh_file_path.c_str());
        return false;
    }
    input.close();

    // 3. Tree 및 shortest_paths 생성
    Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

    auto start_time_point = std::chrono::high_resolution_clock::now();
    geodesic_path = n_io.loadGeodesicWaypointsFromFile(geodesic_waypoints_file_path);
    // 4. 경로 보간: geodesic_path는 이전에 설정된 전역 또는 클래스 멤버
    nrs_path::Waypoints final_waypoints = n_interpolation.interpolateEnd2End(geodesic_path, desired_interval, tmesh, Fx, Fy, Fz);

    // 보간 결과가 비어있으면 오류 처리
    if (final_waypoints.waypoints.empty())
    {
        ROS_ERROR("Interpolation produced no waypoints.");
        delete tree;
        delete shortest_paths;
        return false;
    }

    // 5. 보간 결과를 퍼블리시
    interpolated_waypoints_pub.publish(final_waypoints);

    // 6. 파일 초기화 및 저장
    n_io.clearFile(interpolated_waypoints_file_path);
    n_io.saveWaypointsToFile(final_waypoints, interpolated_waypoints_file_path);

    auto end_time_point = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_point - start_time_point).count();
    std::cout << "Interpolation & Normal smoothing time: " << duration << " s" << std::endl;

    // 메모리 해제
    delete tree;
    delete shortest_paths;

    return true; // 모든 단계가 성공했으므로 true 반환
}

bool nrs_callback::pathDeleteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    n_visualization.deleteMarkers();
    selected_points.clear();
    geodesic_path.waypoints.clear();
    nrs_path::Waypoints empty_waypoints;
    empty_waypoints.waypoints.clear();
    interpolated_waypoints_pub.publish(empty_waypoints);
    ROS_INFO("Path deleted via pathDeleteCallback");

    return true;
}
