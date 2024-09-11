#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/STL_reader.h>
#include <fstream>

// CGAL 관련 타입 정의
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef Kernel::Point_3 Point_3;

// 메쉬 파일에서 엣지 정보를 추출하는 함수
std::vector<geometry_msgs::Point> extractEdgesFromMesh(const Triangle_mesh& mesh) {
    std::vector<geometry_msgs::Point> edges;

    for (auto edge : mesh.edges()) {
        auto halfedge = mesh.halfedge(edge);  // Edge_index에서 Halfedge_index 가져오기
        auto vertex1 = mesh.source(halfedge);  // Halfedge의 source vertex
        auto vertex2 = mesh.target(halfedge);  // Halfedge의 target vertex

        Point_3 p1 = mesh.point(vertex1);
        Point_3 p2 = mesh.point(vertex2);

        geometry_msgs::Point ros_p1, ros_p2;
        ros_p1.x = p1.x(); ros_p1.y = p1.y(); ros_p1.z = p1.z();
        ros_p2.x = p2.x(); ros_p2.y = p2.y(); ros_p2.z = p2.z();

        edges.push_back(ros_p1);
        edges.push_back(ros_p2);
    }   

    return edges;
}

// 메쉬의 엣지를 Rviz에서 시각화하는 함수
void visualizeMeshEdges(ros::Publisher& marker_pub, const std::vector<geometry_msgs::Point>& edges) {
    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = "workpiece";  // 변경 가능
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.ns = "mesh_edges";
    edge_marker.id = 0;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.scale.x = 0.001;  // 테두리 선의 두께

    // 테두리의 색깔 설정 (흰색)
    edge_marker.color.r = 0.0;
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 0.0;
    edge_marker.color.a = 1.0;

    // 엣지 포인트를 마커에 추가
    for (const auto& point : edges) {
        edge_marker.points.push_back(point);
    }

    // 퍼블리시
    marker_pub.publish(edge_marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mesh_edge_visualizer");
    ros::NodeHandle nh;

    // Marker 퍼블리셔 초기화
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_mesh_edge", 1);

    // 메쉬 파일 로드
    std::string mesh_file_path = "/home/nrs/catkin_ws/src/nrs_path_planning/mesh/lid_wrap.stl";
    Triangle_mesh mesh;

    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input || !CGAL::IO::read_STL(input, mesh)) {
        ROS_ERROR("Failed to load mesh file.");
        return -1;
    }

    // 메쉬의 엣지 추출
    std::vector<geometry_msgs::Point> mesh_edges = extractEdgesFromMesh(mesh);

    ros::Rate r(10); // 10 Hz

    while (ros::ok()) {
        // 메쉬 엣지 시각화
        visualizeMeshEdges(marker_pub, mesh_edges);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
