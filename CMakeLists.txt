cmake_minimum_required(VERSION 3.0.2)
project(nrs_path)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)

set(Eigen3_DIR /usr/include/eigen3)


# ROS 필수 패키지 찾기 (roscpp, std_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  image_transport
  cv_bridge
  image_geometry
  geometry_msgs
  realsense2
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
  message_generation
  visualization_msgs
  moveit_ros_planning_interface
)


# 외부 라이브러리 찾기 (PCL, OpenCV, CGAL, VTK, GMP, MPFR)

find_package(VTK REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)


## 서비스 파일 추가
add_service_files(
  FILES
  Command.srv
)

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


###################################
## catkin 패키지 설정 ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge image_geometry
  DEPENDS VTK PCL OpenCV CGAL
  message_runtime
  EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp
)
###################################
## include 디렉토리 설정 ##
###################################
include_directories(
  include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${MPFR_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

###################################
## 빌드 설정 ##
###################################

add_executable(path_projection src/path_projection.cpp)
add_dependencies(path_projection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_projection ${catkin_LIBRARIES} ${CGAL_LIBRARIES})

############################################################################################
link_libraries(${REALSENSE2_FOUND}
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
${OpenCV_LIBS} 
${image_transport_LIBRARIES} 
${cv_bridge_LIBRARIES} 
${image_geometry_LIBRARIES}
${CGAL_LIBRARIES}
${YAML_CPP_LIBRARIES})


add_executable(nrs_node_path_generation
src/nrs_node_path_generation.cpp

src/nrs_geodesic.cpp
src/nrs_vec3d.cpp
src/nrs_interpolation.cpp
src/nrs_visualization.cpp


src/nrs_callback.cpp
src/nrs_io.cpp
src/nrs_math.cpp
)

add_dependencies(nrs_node_path_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(nrs_node_simulation
src/nrs_node_simulation.cpp


src/nrs_geodesic.cpp
src/nrs_vec3d.cpp
src/nrs_interpolation.cpp
src/nrs_visualization.cpp


src/nrs_callback.cpp
src/nrs_io.cpp
src/nrs_math.cpp
)
add_dependencies(nrs_node_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(nrs_node_visualization
src/nrs_node_visualization.cpp


src/nrs_geodesic.cpp
src/nrs_vec3d.cpp
src/nrs_interpolation.cpp
src/nrs_visualization.cpp


src/nrs_callback.cpp
src/nrs_io.cpp
src/nrs_math.cpp
)
add_dependencies(nrs_node_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# catkin_install_python(PROGRAMS
#   data/Ori_path_translation.py
#   data/Ori_path_projection.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# set(LIBIGL_DIR "/home/nrs/libigl")
# set(LIBIGL_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/libigl")
# add_subdirectory(${LIBIGL_DIR} ${LIBIGL_BINARY_DIR})
# include_directories(${LIBIGL_DIR}/include)
# add_executable(yonoo src/yonoo.cpp)
# target_link_libraries(yonoo igl::core)