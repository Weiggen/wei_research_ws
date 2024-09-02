# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "voronoi_cbsa: 13 messages, 0 services")

set(MSG_I_FLAGS "-Ivoronoi_cbsa:/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(voronoi_cbsa_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" "voronoi_cbsa/SensorArray:geometry_msgs/Point:voronoi_cbsa/WeightArray:voronoi_cbsa/Weight:voronoi_cbsa/Sensor"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" ""
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" "voronoi_cbsa/SensorArray:geometry_msgs/Point:voronoi_cbsa/Sensor"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" "voronoi_cbsa/SensorArray:geometry_msgs/Point:voronoi_cbsa/NeighborInfo:voronoi_cbsa/Sensor"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" "voronoi_cbsa/SensorArray:geometry_msgs/Point:voronoi_cbsa/WeightArray:voronoi_cbsa/Weight:voronoi_cbsa/Sensor:voronoi_cbsa/ExchangeData"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" "geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Twist"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" "geometry_msgs/Point:voronoi_cbsa/TargetInfo:geometry_msgs/Vector3:geometry_msgs/Twist"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" "voronoi_cbsa/Sensor"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" ""
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" ""
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" ""
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" "voronoi_cbsa/Weight"
)

get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" NAME_WE)
add_custom_target(_voronoi_cbsa_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "voronoi_cbsa" "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_cpp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
)

### Generating Services

### Generating Module File
_generate_module_cpp(voronoi_cbsa
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(voronoi_cbsa_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(voronoi_cbsa_generate_messages voronoi_cbsa_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_cpp _voronoi_cbsa_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(voronoi_cbsa_gencpp)
add_dependencies(voronoi_cbsa_gencpp voronoi_cbsa_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS voronoi_cbsa_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_eus(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
)

### Generating Services

### Generating Module File
_generate_module_eus(voronoi_cbsa
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(voronoi_cbsa_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(voronoi_cbsa_generate_messages voronoi_cbsa_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_eus _voronoi_cbsa_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(voronoi_cbsa_geneus)
add_dependencies(voronoi_cbsa_geneus voronoi_cbsa_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS voronoi_cbsa_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_lisp(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
)

### Generating Services

### Generating Module File
_generate_module_lisp(voronoi_cbsa
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(voronoi_cbsa_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(voronoi_cbsa_generate_messages voronoi_cbsa_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_lisp _voronoi_cbsa_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(voronoi_cbsa_genlisp)
add_dependencies(voronoi_cbsa_genlisp voronoi_cbsa_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS voronoi_cbsa_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_nodejs(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
)

### Generating Services

### Generating Module File
_generate_module_nodejs(voronoi_cbsa
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(voronoi_cbsa_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(voronoi_cbsa_generate_messages voronoi_cbsa_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_nodejs _voronoi_cbsa_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(voronoi_cbsa_gennodejs)
add_dependencies(voronoi_cbsa_gennodejs voronoi_cbsa_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS voronoi_cbsa_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg"
  "${MSG_I_FLAGS}"
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)
_generate_msg_py(voronoi_cbsa
  "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
)

### Generating Services

### Generating Module File
_generate_module_py(voronoi_cbsa
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(voronoi_cbsa_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(voronoi_cbsa_generate_messages voronoi_cbsa_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg" NAME_WE)
add_dependencies(voronoi_cbsa_generate_messages_py _voronoi_cbsa_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(voronoi_cbsa_genpy)
add_dependencies(voronoi_cbsa_genpy voronoi_cbsa_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS voronoi_cbsa_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/voronoi_cbsa
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(voronoi_cbsa_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/voronoi_cbsa
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(voronoi_cbsa_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/voronoi_cbsa
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(voronoi_cbsa_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/voronoi_cbsa
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(voronoi_cbsa_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/voronoi_cbsa
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(voronoi_cbsa_generate_messages_py geometry_msgs_generate_messages_py)
endif()
