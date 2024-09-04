cmake_minimum_required(VERSION 3.0.2)
project(nrs_vision_rviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  visualization_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

add_message_files(
  FILES
  Waypoint.msg
  Waypoints.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_executable(visualize_mesh src/visualize_mesh.cpp)
target_link_libraries(visualize_mesh ${catkin_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(point_publisher src/point_publisher.cpp)
target_link_libraries(point_publisher ${catkin_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(keyboard_listener src/keyboard_listener.cpp)
target_link_libraries(keyboard_listener ${catkin_LIBRARIES})

add_executable(path_generator src/path_generator.cpp)
add_dependencies(path_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_generator ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(geodesic_hermite_path_generator src/geodesic_hermite_path_generator.cpp)
add_dependencies(geodesic_hermite_path_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geodesic_hermite_path_generator ${catkin_LIBRARIES}  ${CGAL_LIBRARIES})

add_executable(path_visualizer src/path_visualizer.cpp)
add_dependencies(path_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_visualizer ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(moveit_planner src/moveit_planner.cpp)
add_dependencies(moveit_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(waypoints_interpolator src/waypoints_interpolator.cpp)
add_dependencies(waypoints_interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoints_interpolator ${catkin_LIBRARIES} ${CGAL_LIBRARIES})
