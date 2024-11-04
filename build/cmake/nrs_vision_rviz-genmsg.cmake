# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nrs_vision_rviz: 2 messages, 0 services")

set(MSG_I_FLAGS "-Inrs_vision_rviz:/home/nrs/catkin_ws/src/nrs_vision_rviz/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nrs_vision_rviz_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" NAME_WE)
add_custom_target(_nrs_vision_rviz_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nrs_vision_rviz" "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" "geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" NAME_WE)
add_custom_target(_nrs_vision_rviz_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nrs_vision_rviz" "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" "geometry_msgs/Quaternion:nrs_vision_rviz/Waypoint:geometry_msgs/Point:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nrs_vision_rviz
)
_generate_msg_cpp(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nrs_vision_rviz
)

### Generating Services

### Generating Module File
_generate_module_cpp(nrs_vision_rviz
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nrs_vision_rviz
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nrs_vision_rviz_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nrs_vision_rviz_generate_messages nrs_vision_rviz_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_cpp _nrs_vision_rviz_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_cpp _nrs_vision_rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nrs_vision_rviz_gencpp)
add_dependencies(nrs_vision_rviz_gencpp nrs_vision_rviz_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nrs_vision_rviz_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nrs_vision_rviz
)
_generate_msg_eus(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nrs_vision_rviz
)

### Generating Services

### Generating Module File
_generate_module_eus(nrs_vision_rviz
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nrs_vision_rviz
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(nrs_vision_rviz_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(nrs_vision_rviz_generate_messages nrs_vision_rviz_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_eus _nrs_vision_rviz_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_eus _nrs_vision_rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nrs_vision_rviz_geneus)
add_dependencies(nrs_vision_rviz_geneus nrs_vision_rviz_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nrs_vision_rviz_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nrs_vision_rviz
)
_generate_msg_lisp(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nrs_vision_rviz
)

### Generating Services

### Generating Module File
_generate_module_lisp(nrs_vision_rviz
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nrs_vision_rviz
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nrs_vision_rviz_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nrs_vision_rviz_generate_messages nrs_vision_rviz_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_lisp _nrs_vision_rviz_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_lisp _nrs_vision_rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nrs_vision_rviz_genlisp)
add_dependencies(nrs_vision_rviz_genlisp nrs_vision_rviz_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nrs_vision_rviz_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nrs_vision_rviz
)
_generate_msg_nodejs(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nrs_vision_rviz
)

### Generating Services

### Generating Module File
_generate_module_nodejs(nrs_vision_rviz
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nrs_vision_rviz
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(nrs_vision_rviz_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(nrs_vision_rviz_generate_messages nrs_vision_rviz_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_nodejs _nrs_vision_rviz_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_nodejs _nrs_vision_rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nrs_vision_rviz_gennodejs)
add_dependencies(nrs_vision_rviz_gennodejs nrs_vision_rviz_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nrs_vision_rviz_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nrs_vision_rviz
)
_generate_msg_py(nrs_vision_rviz
  "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nrs_vision_rviz
)

### Generating Services

### Generating Module File
_generate_module_py(nrs_vision_rviz
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nrs_vision_rviz
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nrs_vision_rviz_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nrs_vision_rviz_generate_messages nrs_vision_rviz_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoint.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_py _nrs_vision_rviz_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nrs/catkin_ws/src/nrs_vision_rviz/msg/Waypoints.msg" NAME_WE)
add_dependencies(nrs_vision_rviz_generate_messages_py _nrs_vision_rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nrs_vision_rviz_genpy)
add_dependencies(nrs_vision_rviz_genpy nrs_vision_rviz_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nrs_vision_rviz_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nrs_vision_rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nrs_vision_rviz
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(nrs_vision_rviz_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(nrs_vision_rviz_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nrs_vision_rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nrs_vision_rviz
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(nrs_vision_rviz_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(nrs_vision_rviz_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nrs_vision_rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nrs_vision_rviz
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(nrs_vision_rviz_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(nrs_vision_rviz_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nrs_vision_rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nrs_vision_rviz
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(nrs_vision_rviz_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(nrs_vision_rviz_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nrs_vision_rviz)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nrs_vision_rviz\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nrs_vision_rviz
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(nrs_vision_rviz_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(nrs_vision_rviz_generate_messages_py std_msgs_generate_messages_py)
endif()
