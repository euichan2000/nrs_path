cmake_minimum_required(VERSION 3.0.2)
project(nrs_ver2)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  visualization_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## PCL 라이브러리 포함
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Eigen 경로 추가
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## CGAL 라이브러리 경로 설정
find_package(CGAL REQUIRED)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})


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
## catkin 패키지 설정
catkin_package(
  CATKIN_DEPENDS roscpp # roscpp와 rospy를 종속성으로 포함
  message_runtime
  EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp
)





## CGAL 포함 디렉토리와 라이브러리 추가
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS} # /usr/local/include/CGAL 포함
  ${PROJECT_SOURCE_DIR}/include # /include 추가
)

add_executable(reshape_stl src/reshape_stl.cpp)
target_link_libraries(reshape_stl
  ${catkin_LIBRARIES}
  ${CGAL_LIBRARIES}
)

add_executable(point_visualizer src/point_visualizer.cpp)
target_link_libraries(point_visualizer ${catkin_LIBRARIES})

add_executable(keyboard_listener src/keyboard_listener.cpp)
target_link_libraries(keyboard_listener ${catkin_LIBRARIES})

add_executable(path_generator src/path_generator.cpp)
add_dependencies(path_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_generator ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(path_visualizer src/path_visualizer.cpp)
add_dependencies(path_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_visualizer ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(waypoints_interpolator src/waypoints_interpolator.cpp)
add_dependencies(waypoints_interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoints_interpolator ${catkin_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(plane_detection src/plane_detection.cpp ${SOURCES})
target_link_libraries(plane_detection
  ${catkin_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(path_transformer src/path_transformer.cpp)
add_dependencies(path_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_transformer ${catkin_LIBRARIES} yaml-cpp)

add_executable(moveit_planner src/moveit_planner.cpp)
add_dependencies(moveit_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_planner ${catkin_LIBRARIES})