#include <ros/ros.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Real_timer.h>

#include <iostream>
#include <string>

namespace PMP = CGAL::Polygon_mesh_processing;

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = K::Point_3;

using Mesh = CGAL::Surface_mesh<Point_3>;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reshape_stl_node");
    ros::NodeHandle nh;

    // Retrieve parameters from params.yaml
    ros::param::set("/reshape_done", false);
    std::string source_file, target_file;
    if (!nh.getParam("reshape_stl_node/source_file", source_file))
    {
        ROS_ERROR("Failed to get 'source_file' parameter.");
        return EXIT_FAILURE;
    }

    if (!nh.getParam("reshape_stl_node/target_file", target_file))
    {
        ROS_ERROR("Failed to get 'target_file' parameter.");
        return EXIT_FAILURE;
    }

    Mesh mesh;
    if (!PMP::IO::read_polygon_mesh(source_file, mesh) || is_empty(mesh) || !is_triangle_mesh(mesh))
    {
        ROS_ERROR("Invalid input: %s", source_file.c_str());
        return EXIT_FAILURE;
    }

    std::cout << "Input: " << num_vertices(mesh) << " vertices, " << num_faces(mesh) << " faces" << std::endl;

    // Compute the alpha and offset values
    const double relative_alpha = (argc > 2) ? std::stod(argv[2]) : 100.;
    const double relative_offset = (argc > 3) ? std::stod(argv[3]) : 1000.;

    CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(mesh);
    const double diag_length = std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
                                         CGAL::square(bbox.ymax() - bbox.ymin()) +
                                         CGAL::square(bbox.zmax() - bbox.zmin()));

    const double alpha = diag_length / relative_alpha;
    const double offset = diag_length / relative_offset;
    ROS_INFO("alpha: %f, offset: %f", alpha, offset);

    // Construct the wrap
    CGAL::Real_timer t;
    t.start();

    Mesh wrap;
    CGAL::alpha_wrap_3(mesh, alpha, offset, wrap);

    t.stop();
    std::cout << "Result: " << num_vertices(wrap) << " vertices, " << num_faces(wrap) << " faces" << std::endl;
    std::cout << "Took " << t.time() << " s." << std::endl;
    // Save the result
    ROS_INFO("Writing to %s", target_file.c_str());
    CGAL::IO::write_polygon_mesh(target_file, wrap, CGAL::parameters::stream_precision(17));
    ros::param::set("/reshape_done", true);

    return EXIT_SUCCESS;
}
