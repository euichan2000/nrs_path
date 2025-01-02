#include <ros/ros.h>
#include <ros/console.h>
#include <fstream>
#include <string>
#include <iostream>
#include <filesystem>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/IO/PLY.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Point_set.h>
#include <boost/iterator/function_output_iterator.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <random>

// Typedefs
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using FT = typename Kernel::FT;
using Point_3 = typename Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Point_set = CGAL::Point_set_3<Point_3>;
using Output_range = CGAL::Point_set_3<Point_3>;
using Point_map = typename Point_set::Point_map;
using Normal_map = typename Point_set::Vector_map;

using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region_for_point_set<Point_set>;
using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query_for_point_set<Point_set>;
using Sorting = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_sorting_for_point_set<Point_set, Neighbor_query>;
using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

namespace fs = std::filesystem;

// Custom output iterator to handle region processing
struct PlaneDetectionOutputIterator
{
    Point_set::Property_map<std::size_t> &plane_index_map;
    Point_set::Property_map<Kernel::Vector_3> &normal_vector_map;
    Point_set::Property_map<unsigned int> &red_map;
    Point_set::Property_map<unsigned int> &green_map;
    Point_set::Property_map<unsigned int> &blue_map;
    std::size_t &plane_id;

    std::mt19937 rng;
    std::uniform_int_distribution<int> dist;

    PlaneDetectionOutputIterator(Point_set::Property_map<std::size_t> &index_map,
                                 Point_set::Property_map<Kernel::Vector_3> &normal_map,
                                 Point_set::Property_map<unsigned int> &red,
                                 Point_set::Property_map<unsigned int> &green,
                                 Point_set::Property_map<unsigned int> &blue,
                                 std::size_t &id)
        : plane_index_map(index_map),
          normal_vector_map(normal_map),
          red_map(red),
          green_map(green),
          blue_map(blue),
          plane_id(id),
          rng(std::random_device{}()),
          dist(0, 255) {}

    PlaneDetectionOutputIterator &operator++() { return *this; }
    PlaneDetectionOutputIterator &operator++(int) { return *this; }

    void operator()(const std::pair<Region_type::Primitive, std::vector<Region_type::Item>> &region)
    {
        Kernel::Vector_3 normal = region.first.orthogonal_vector();
        // Generate a random color for the plane
        const unsigned int r = static_cast<unsigned int>(dist(rng));
        const unsigned int g = static_cast<unsigned int>(dist(rng));
        const unsigned int b = static_cast<unsigned int>(dist(rng));
        for (const auto &index : region.second)
        {
            plane_index_map[index] = plane_id;
            normal_vector_map[index] = normal;
            red_map[index] = r;
            green_map[index] = g;
            blue_map[index] = b;
        }
        plane_id++;
    }
};

pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k_neighbors)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setKSearch(k_neighbors);
    normal_estimation.compute(*normals);

    return normals;
}

// XYZ 파일 저장
bool saveXYZ(const std::string &output_xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    if (cloud->size() != normals->size())
    {

        return false;
    }

    std::ofstream output_file(output_xyz);
    if (!output_file.is_open())
    {

        return false;
    }

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const auto &point = cloud->points[i];
        const auto &normal = normals->points[i];

        output_file << point.x << " " << point.y << " " << point.z << " "
                    << normal.normal_x << " " << normal.normal_y << " " << normal.normal_z << "\n";
    }

    output_file.close();

    return true;
}

// PCD 파일을 XYZ로 변환
bool convert(const std::string &input_pcd, const std::string &output_xyz, int k_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // PCD 파일 읽기
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *cloud) == -1)
    {

        return false;
    }

    // 법선 계산

    auto normals = computeNormal(cloud, k_neighbors);
    if (!normals)
    {

        return false;
    }
    return saveXYZ(output_xyz, cloud, normals);
}

bool detect_planes(const std::string &input_pcd, const std::string &output_ply)
{
    // 1. PCD 파일을 XYZ로 변환
    std::string intermediate_xyz = input_pcd.substr(0, input_pcd.find_last_of('.')) + ".xyz";
    convert(input_pcd, intermediate_xyz, 10);
    ROS_INFO("Intermediate XYZ file saved: %s", intermediate_xyz.c_str());

    // 2. XYZ 파일을 CGAL Point_set으로 로드
    std::ifstream in(intermediate_xyz);
    if (!in)
    {
        ROS_ERROR("Failed to open intermediate XYZ file: %s", intermediate_xyz.c_str());
        return false;
    }

    CGAL::IO::set_ascii_mode(in);
    const bool with_normal_map = true;
    Point_set point_set(with_normal_map);
    in >> point_set;
    in.close();
    ROS_INFO("* Number of input points: %lu", point_set.size());

    // 3. 속성 맵 추가
    auto plane_index_map = point_set.add_property_map<std::size_t>("plane_index", static_cast<std::size_t>(-1)).first;
    auto normal_vector_map = point_set.add_property_map<Kernel::Vector_3>("normal_vector", Kernel::Vector_3(0, 0, 0)).first;
    auto red_map = point_set.add_property_map<unsigned int>("red", 0).first;
    auto green_map = point_set.add_property_map<unsigned int>("green", 0).first;
    auto blue_map = point_set.add_property_map<unsigned int>("blue", 0).first;

    // 4. Region Growing 알고리즘
    const std::size_t k = 12;
    const FT max_distance = FT(2);
    const FT max_angle = FT(15);
    const std::size_t min_region_size = 100;

    Neighbor_query neighbor_query = CGAL::Shape_detection::Point_set::make_k_neighbor_query(
        point_set, CGAL::parameters::k_neighbors(k));

    Sorting sorting = CGAL::Shape_detection::Point_set::make_least_squares_plane_fit_sorting(point_set, neighbor_query);
    sorting.sort();

    Region_type region_type = CGAL::Shape_detection::Point_set::make_least_squares_plane_fit_region(
        point_set,
        CGAL::parameters::maximum_distance(max_distance)
            .maximum_angle(max_angle)
            .minimum_region_size(min_region_size));

    Region_growing region_growing(point_set, sorting.ordered(), neighbor_query, region_type);

    // 5. 평면 검출
    std::size_t plane_id = 0;
    PlaneDetectionOutputIterator output_iterator(plane_index_map, normal_vector_map, red_map, green_map, blue_map, plane_id);
    region_growing.detect(boost::make_function_output_iterator(output_iterator));

    ROS_INFO("* Number of found planes: %lu", plane_id);

    // 6. PLY 파일 저장
    std::ofstream out(output_ply);
    if (!out)
    {
        ROS_ERROR("Failed to open output PLY file: %s", output_ply.c_str());
        return false;
    }
    out << point_set;
    out.close();
    ROS_INFO("Saved to %s", output_ply.c_str());

    return true;
}

bool LoadCustomPLYToPointSet(const std::string &ply_file, Point_set &point_set)
{
    std::ifstream file(ply_file);
    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open PLY file: " << ply_file << std::endl;
        return false;
    }

    std::string line;
    bool header_parsed = false;
    std::size_t vertex_count = 0;

    // Parse the header
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;

        if (keyword == "element" && line.find("vertex") != std::string::npos)
        {
            iss >> keyword >> vertex_count;
        }
        else if (keyword == "end_header")
        {
            header_parsed = true;
            break;
        }
    }

    if (!header_parsed)
    {
        std::cerr << "Error: Invalid PLY file format (missing header)." << std::endl;
        return false;
    }

    // Add property maps
    auto plane_index_map = point_set.add_property_map<std::size_t>("plane_index", -1).first;
    auto normal_map = point_set.add_property_map<Vector_3>("normal_vector", Vector_3(0, 0, 0)).first;
    auto red_map = point_set.add_property_map<unsigned int>("red", 0).first;
    auto green_map = point_set.add_property_map<unsigned int>("green", 0).first;
    auto blue_map = point_set.add_property_map<unsigned int>("blue", 0).first;

    // Parse vertex data
    std::size_t line_count = 0;
    while (std::getline(file, line) && line_count < vertex_count)
    {
        std::istringstream iss(line);
        double x, y, z, nx, ny, nz;
        std::size_t plane_index;
        unsigned int r, g, b;

        // Read the data
        if (!(iss >> x >> y >> z >> nx >> ny >> nz >> plane_index >> r >> g >> b))
        {
            std::cerr << "Error: Malformed vertex data at line " << line_count + 1 << "." << std::endl;
            continue;
        }

        // Insert point and set properties
        auto it = point_set.insert(Point_3(x, y, z)); // `insert` returns an iterator

        // Use `operator[]` to set properties
        normal_map[*it] = Vector_3(nx, ny, nz);
        plane_index_map[*it] = plane_index;
        red_map[*it] = r;
        green_map[*it] = g;
        blue_map[*it] = b;

        ++line_count;
    }

    if (line_count != vertex_count)
    {
        std::cerr << "Warning: Expected " << vertex_count << " vertices but loaded " << line_count << "." << std::endl;
    }

    std::cout << "Loaded " << point_set.size() << " points from " << ply_file << std::endl;
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "plane_detection_node");
    ros::NodeHandle nh; // Private namespace for this node
    std::string input_pcd1, input_pcd2, output_ply1, output_ply2;
    // 템플릿에서 $(file_name)을 실제 값으로 대체
    nh.getParam("plane_detection_node/input_pcd1", input_pcd1);
    nh.getParam("plane_detection_node/input_pcd2", input_pcd2);
    nh.getParam("plane_detection_node/output_ply1", output_ply1);
    nh.getParam("plane_detection_node/output_ply2", output_ply2);

    // Check if target file already exists
    if (fs::exists(output_ply1) && fs::exists(output_ply2))
    {
        ROS_WARN("file '%s' & '%s' already exists. Exiting.", output_ply1.c_str(), output_ply1.c_str());
        ros::param::set("/plane_detection_done", true);
        return EXIT_SUCCESS;
    }

    // 첫 번째 파일 처리
    if (!detect_planes(input_pcd1, output_ply1))
    {
        ROS_ERROR("Failed to process first file.");
        return EXIT_FAILURE;
    }

    // 두 번째 파일 처리
    if (!detect_planes(input_pcd2, output_ply2))
    {
        ROS_ERROR("Failed to process second file.");
        return EXIT_FAILURE;
    }

    ROS_INFO("Plane detection completed for both files.");

    nh.setParam("/plane_detection_done", true);
    ROS_INFO("Plane detection done");

    return EXIT_SUCCESS;
}
