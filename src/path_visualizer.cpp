#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/STL_reader.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <iterator>

#include <nrs_vision_rviz/Waypoints.h>
#include <nrs_vision_rviz/Waypoint.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;
typedef Traits::Point_3 Point_3;
typedef boost::graph_traits<Triangle_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Triangle_mesh>::face_descriptor face_descriptor;

class PathVisualizer
{
public:
    PathVisualizer()
    {

        // mesh 파일 로드
        std::string mesh_file = "/home/nrs/catkin_ws/src/nrs_vision_rviz/mesh/fenda_wrap.stl";
        std::ifstream input(mesh_file, std::ios::binary);

        if (!input || !read_stl_file(input, tmesh))
        {
            ROS_ERROR("Error: cannot read the file %s", mesh_file.c_str());
        }

        shortest_paths = new Surface_mesh_shortest_path(tmesh);

        sub = n.subscribe("/clicked_point", 10, &PathVisualizer::pointCallback, this);
        keyboard_sub = n.subscribe("moveit_command", 10, &PathVisualizer::keyboardCallback, this);
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_path", 10);
        // axis_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_axes", 10);
        waypoints_pub = n.advertise<nrs_vision_rviz::Waypoints>("waypoints_with_normals", 10);
    }

    void keyboardCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "start")
        {
            publishWaypoints(); // 여기에 publishWaypoints 함수 호출 추가
        }
    }
    // 새로운 publishWaypoints 함수 추가
    void publishWaypoints()
    {
        waypoints_pub.publish(waypoints_msg);
        ROS_INFO("Waypoints published.");
    }
    void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        Point_3 point(msg->point.x, msg->point.y, msg->point.z);
        face_descriptor face;
        Surface_mesh_shortest_path::Barycentric_coordinates location;
        if (locate_face_and_point(point, face, location))
        {
            if (!points.empty())
            {
                previous_face = current_face;
                previous_location = current_location;
            }
            points.push_back(point);
            faces.push_back(face);
            locations.push_back(location);

            current_face = face;
            current_location = location;

            if (points.size() > 1)
            {
                visualizePath();
            }
        }
        else
        {
            ROS_ERROR("Point is not on the mesh.");
        }
    }
    bool read_stl_file(std::ifstream &input, Triangle_mesh &mesh)
    {
        std::vector<Kernel::Point_3> points;
        std::vector<std::array<std::size_t, 3>> triangles;
        if (!CGAL::read_STL(input, points, triangles))
        {
            ROS_ERROR("Failed to read STL file.");
            return false;
        }

        std::map<std::size_t, vertex_descriptor> index_to_vertex;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            index_to_vertex[i] = mesh.add_vertex(points[i]);
        }

        for (const auto &t : triangles)
        {
            mesh.add_face(index_to_vertex[t[0]], index_to_vertex[t[1]], index_to_vertex[t[2]]);
        }
        ROS_INFO("Successfully read STL file.");
        return true;
    }

    bool locate_face_and_point(const Point_3 &point, face_descriptor &face, Surface_mesh_shortest_path::Barycentric_coordinates &location)
    {
        typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh> Primitive;
        typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
        typedef CGAL::AABB_tree<AABB_traits> Tree;

        // Check if tmesh is empty
        if (tmesh.faces().empty())
        {
            ROS_ERROR("Mesh is empty, cannot build AABB tree.");
            return false;
        }

        Tree tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
        tree.build();

        // locate_with_AABB_tree 함수를 사용하여 점을 메쉬 상의 가장 가까운 얼굴에 위치시킵니다.
        auto result = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(point, tree, tmesh);

        // result는 항상 pair 타입이므로 유효성 검사 불필요
        face = result.first;
        location = result.second;
        return true;
    }

    void visualizePath()
    {
        if (points.size() < 2)
            return;

        std::vector<Point_3> complete_path;
        std::vector<Point_3> path_segment;

        line_strip.header.frame_id = "base_link";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "path";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.005; // 선의 굵기 설정
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;

        shortest_paths->add_source_point(current_face, current_location);
        shortest_paths->shortest_path_points_to_source_points(previous_face, previous_location, std::back_inserter(path_segment));
        complete_path.insert(complete_path.end(), path_segment.begin(), path_segment.end());
        shortest_paths->remove_all_source_points();

        for (size_t i = 0; i < complete_path.size(); i++)
        {
            nrs_vision_rviz::Waypoint waypoint_msg;
            Point_3 point = complete_path[i];

            // 새로운 face를 찾기 위한 로직 추가
            face_descriptor face;
            Surface_mesh_shortest_path::Barycentric_coordinates location;
            if (!locate_face_and_point(point, face, location))
            {
                ROS_ERROR("Failed to locate face and point");
                continue;
            }

            Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, tmesh);

            geometry_msgs::Point ros_point;
            ros_point.x = point.x();
            ros_point.y = point.y();
            ros_point.z = point.z();

            geometry_msgs::Vector3 ros_normal;
            ros_normal.x = -normal.x();
            ros_normal.y = -normal.y();
            ros_normal.z = -normal.z();

            waypoint_msg.point = ros_point;
            waypoint_msg.normal = ros_normal;
            waypoints_msg.waypoints.push_back(waypoint_msg);

            // Add point to line strip for visualization
            line_strip.points.push_back(ros_point);
        }

        marker_pub.publish(line_strip);
        if (publish == true)
        {
            waypoints_pub.publish(waypoints_msg);
        }

        // for (size_t i = 0; i < complete_path.size(); i=i+20)
        // {
        //     Point_3 point = complete_path[i];
        //     face_descriptor face = faces[i];

        //     Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, tmesh);
        //     Kernel::Vector_3 direction = Kernel::Vector_3(1, 0, 0); // x 방향은 고정된 값 사용
        //     Kernel::Vector_3 y_direction = CGAL::cross_product(normal, direction);

        //     publishAxis(point, direction, y_direction, normal);
        // }
    }

    // void publishAxis(const Point_3 &position, const Kernel::Vector_3 &x_axis, const Kernel::Vector_3 &y_axis, const Kernel::Vector_3 &z_axis)
    // {
    //     // Create markers for x, y, z axes
    //     visualization_msgs::Marker x_marker, y_marker, z_marker;

    //     x_marker.header.frame_id = y_marker.header.frame_id = z_marker.header.frame_id = "map";
    //     x_marker.header.stamp = y_marker.header.stamp = z_marker.header.stamp = ros::Time::now();
    //     x_marker.ns = y_marker.ns = z_marker.ns = "axes";
    //     x_marker.action = y_marker.action = z_marker.action = visualization_msgs::Marker::ADD;
    //     x_marker.pose.orientation.w = y_marker.pose.orientation.w = z_marker.pose.orientation.w = 1.0;
    //     x_marker.type = y_marker.type = z_marker.type = visualization_msgs::Marker::ARROW;
    //     x_marker.scale.x = y_marker.scale.x = z_marker.scale.x = 0.01; // Arrow shaft diameter
    //     x_marker.scale.y = y_marker.scale.y = z_marker.scale.y = 0.02; // Arrow head diameter
    //     x_marker.scale.z = y_marker.scale.z = z_marker.scale.z = 0.02; // Arrow head length

    //     // Set colors for x, y, z axes
    //     x_marker.color.r = 1.0;
    //     x_marker.color.g = 0.0;
    //     x_marker.color.b = 0.0;
    //     x_marker.color.a = 1.0;
    //     y_marker.color.r = 0.0;
    //     y_marker.color.g = 1.0;
    //     y_marker.color.b = 0.0;
    //     y_marker.color.a = 1.0;
    //     z_marker.color.r = 0.0;
    //     z_marker.color.g = 0.0;
    //     z_marker.color.b = 1.0;
    //     z_marker.color.a = 1.0;

    //     double axis_length = 0.1; // 축 길이를 0.1로 설정

    //     geometry_msgs::Point start, end_x, end_y, end_z;
    //     start.x = position.x();
    //     start.y = position.y();
    //     start.z = position.z();
    //     end_x.x = start.x + axis_length * x_axis.x();
    //     end_x.y = start.y + axis_length * x_axis.y();
    //     end_x.z = start.z + axis_length * x_axis.z();
    //     end_y.x = start.x + axis_length * y_axis.x();
    //     end_y.y = start.y + axis_length * y_axis.y();
    //     end_y.z = start.z + axis_length * y_axis.z();
    //     end_z.x = start.x + axis_length * z_axis.x();
    //     end_z.y = start.y + axis_length * z_axis.y();
    //     end_z.z = start.z + axis_length * z_axis.z();

    //     x_marker.points.push_back(start);
    //     x_marker.points.push_back(end_x);
    //     y_marker.points.push_back(start);
    //     y_marker.points.push_back(end_y);
    //     z_marker.points.push_back(start);
    //     z_marker.points.push_back(end_z);

    //     axis_marker_pub.publish(x_marker);
    //     axis_marker_pub.publish(y_marker);
    //     axis_marker_pub.publish(z_marker);
    // }

private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    // ros::Publisher axis_marker_pub;
    ros::Publisher waypoints_pub;
    ros::Subscriber sub;
    ros::Subscriber keyboard_sub;

    std::vector<Point_3> points;
    std::vector<face_descriptor> faces;
    std::vector<Surface_mesh_shortest_path::Barycentric_coordinates> locations;

    Triangle_mesh tmesh;
    Surface_mesh_shortest_path *shortest_paths;
    face_descriptor previous_face, current_face;
    Surface_mesh_shortest_path::Barycentric_coordinates previous_location, current_location;
    visualization_msgs::Marker line_strip;
    nrs_vision_rviz::Waypoints waypoints_msg;
    int axis_id = 0;
    bool publish = false;
};
// CGAL 타입 정의

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_visualizer");
    PathVisualizer p;
    ros::spin();
}