#include "path.h"
#include "cgal_related.h"

void path::generate_Geodesic_Path(const std::vector<geometry_msgs::Point> &points)
{

    std::vector<Point_3> complete_path;
    std::vector<Point_3> path_segment;

    for (size_t i = 1; i < points.size(); ++i)
    {
        Point_3 start(points[i - 1].x, points[i - 1].y, points[i - 1].z);
        Point_3 end(points[i].x, points[i].y, points[i].z);

        face_descriptor start_face, end_face;
        Surface_mesh_shortest_path::Barycentric_coordinates start_location, end_location;

        locate_face_and_point(start, start_face, start_location);
        locate_face_and_point(end, end_face, end_location);
        shortest_paths->add_source_point(end_face, end_location);
        shortest_paths->shortest_path_points_to_source_points(start_face, start_location, std::back_inserter(path_segment));
        shortest_paths->remove_all_source_points();
        complete_path.clear();
        complete_path.insert(complete_path.end(), path_segment.begin(), path_segment.end());
        shortest_paths->remove_all_source_points();
    }

    std::vector<geometry_msgs::Point> path_points;
    for (const auto &point : complete_path)
    {
        geometry_msgs::Point ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_point.z = point.z();
        path_points.push_back(ros_point);
    }

    ROS_INFO("Generated geodesic path with %zu points", path_points.size());
    waypoints_msg.waypoints = convertToWaypoints(path_points);
}

void path::generate_B_Spline_Path(const std::vector<geometry_msgs::Point> &points)
{
    unsigned int maxSteps = 2;
    double minChange = std::numeric_limits<double>::epsilon();
    std::vector<geometry_msgs::Point> smooth_path;

    if (points.size() < 2)
        return;

    auto state_space(std::make_shared<ob::RealVectorStateSpace>(3));
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    state_space->setBounds(bounds);

    og::SimpleSetup ss(state_space);
    ss.setStateValidityChecker(std::make_shared<ValidityChecker>(ss.getSpaceInformation()));

    og::PathGeometric full_path(ss.getSpaceInformation());

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        ob::ScopedState<> start(state_space), goal(state_space);

        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = points[i].x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = points[i].y;
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = points[i].z;

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = points[i + 1].x;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = points[i + 1].y;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = points[i + 1].z;

        ss.setStartAndGoalStates(start, goal);

        auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        ss.setPlanner(planner);

        ob::PlannerStatus solved = ss.solve(1.0);

        if (solved)
        {
            ss.simplifySolution();
            og::PathGeometric path = ss.getSolutionPath();

            if (i == 0)
            {
                full_path.append(path);
            }
            else
            {
                for (size_t j = 1; j < path.getStateCount(); ++j)
                {
                    full_path.append(path.getState(j));
                }
            }
        }
    }

    og::PathSimplifier simplifier(ss.getSpaceInformation());
    simplifier.smoothBSpline(full_path, maxSteps, minChange);

    for (size_t i = 0; i < full_path.getStateCount(); ++i)
    {
        const ob::RealVectorStateSpace::StateType *state = full_path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        geometry_msgs::Point p;
        p.x = state->values[0];
        p.y = state->values[1];
        p.z = state->values[2];
        smooth_path.push_back(p);
    }
    std::vector<geometry_msgs::Point> projected_path = projectPointsOntoMesh(smooth_path);
    waypoints_msg.waypoints = convertToWaypoints(smooth_path);
}

void path::generate_Catmull_Rom_Path(const std::vector<geometry_msgs::Point> &points)
{
    std::vector<geometry_msgs::Point> smooth_path;
    if (points.size() < 4)
        return; // Need at least 4 points for Catmull-Rom spline

    for (size_t i = 1; i < points.size() - 2; ++i)
    {
        geometry_msgs::Point p0 = points[i - 1];
        geometry_msgs::Point p1 = points[i];
        geometry_msgs::Point p2 = points[i + 1];
        geometry_msgs::Point p3 = points[i + 2];

        for (double t = 0; t <= 1; t += 0.05)
        {
            double t2 = t * t;
            double t3 = t2 * t;

            geometry_msgs::Point p;
            p.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
            p.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
            p.z = 0.5 * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 + (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3);

            smooth_path.push_back(p);
        }
    }

    std::vector<geometry_msgs::Point> projected_path = projectPointsOntoMesh(smooth_path);
    waypoints_msg.waypoints = convertToWaypoints(projected_path);
}

void path::generate_Hermite_Spline_path(const std::vector<std::vector<Eigen::Vector3d>> &bezier_control_points, int steps)
{
    std::vector<Eigen::Vector3d> hermite_spline;
    int i = 0;

    for (const auto &control_points : bezier_control_points)
    {
        std::cout << "generating Spline bewteen point[" << i << "] and point[" << i + 1 << "]" << std::endl;
        std::vector<Eigen::Vector3d> curve_points = computeGeodesicBezierCurvePoints(control_points, mesh, steps, marker_array, marker_id);
        hermite_spline.insert(hermite_spline.end(), curve_points.begin(), curve_points.end());
        i += 1;
    }

    std::vector<geometry_msgs::Point> path_points;
    for (const auto &point : hermite_spline)
    {
        geometry_msgs::Point ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_point.z = point.z();
        path_points.push_back(ros_point);
    }

    ROS_INFO("Generated Hermite_Spline path with %zu points", path_points.size());
    waypoints_msg.waypoints = convertToWaypoints(path_points, mesh);
}