# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "msckf_vio: 3 messages, 0 services")

set(MSG_I_FLAGS "-Imsckf_vio:/home/gaofan/s-msckf/src/msckf_vio/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(msckf_vio_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" NAME_WE)
add_custom_target(_msckf_vio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msckf_vio" "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" NAME_WE)
add_custom_target(_msckf_vio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msckf_vio" "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" ""
)

get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" NAME_WE)
add_custom_target(_msckf_vio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msckf_vio" "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" "msckf_vio/FeatureMeasurement:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msckf_vio
)
_generate_msg_cpp(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msckf_vio
)
_generate_msg_cpp(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msckf_vio
)

### Generating Services

### Generating Module File
_generate_module_cpp(msckf_vio
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msckf_vio
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(msckf_vio_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(msckf_vio_generate_messages msckf_vio_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_cpp _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_cpp _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_cpp _msckf_vio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msckf_vio_gencpp)
add_dependencies(msckf_vio_gencpp msckf_vio_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msckf_vio_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msckf_vio
)
_generate_msg_eus(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msckf_vio
)
_generate_msg_eus(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msckf_vio
)

### Generating Services

### Generating Module File
_generate_module_eus(msckf_vio
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msckf_vio
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(msckf_vio_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(msckf_vio_generate_messages msckf_vio_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_eus _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_eus _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_eus _msckf_vio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msckf_vio_geneus)
add_dependencies(msckf_vio_geneus msckf_vio_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msckf_vio_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msckf_vio
)
_generate_msg_lisp(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msckf_vio
)
_generate_msg_lisp(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msckf_vio
)

### Generating Services

### Generating Module File
_generate_module_lisp(msckf_vio
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msckf_vio
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(msckf_vio_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(msckf_vio_generate_messages msckf_vio_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_lisp _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_lisp _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_lisp _msckf_vio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msckf_vio_genlisp)
add_dependencies(msckf_vio_genlisp msckf_vio_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msckf_vio_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msckf_vio
)
_generate_msg_nodejs(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msckf_vio
)
_generate_msg_nodejs(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msckf_vio
)

### Generating Services

### Generating Module File
_generate_module_nodejs(msckf_vio
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msckf_vio
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(msckf_vio_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(msckf_vio_generate_messages msckf_vio_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_nodejs _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_nodejs _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_nodejs _msckf_vio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msckf_vio_gennodejs)
add_dependencies(msckf_vio_gennodejs msckf_vio_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msckf_vio_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio
)
_generate_msg_py(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio
)
_generate_msg_py(msckf_vio
  "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio
)

### Generating Services

### Generating Module File
_generate_module_py(msckf_vio
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(msckf_vio_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(msckf_vio_generate_messages msckf_vio_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/TrackingInfo.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_py _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/FeatureMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_py _msckf_vio_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gaofan/s-msckf/src/msckf_vio/msg/CameraMeasurement.msg" NAME_WE)
add_dependencies(msckf_vio_generate_messages_py _msckf_vio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msckf_vio_genpy)
add_dependencies(msckf_vio_genpy msckf_vio_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msckf_vio_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msckf_vio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msckf_vio
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(msckf_vio_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msckf_vio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msckf_vio
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(msckf_vio_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msckf_vio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msckf_vio
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(msckf_vio_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msckf_vio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msckf_vio
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(msckf_vio_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msckf_vio
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(msckf_vio_generate_messages_py std_msgs_generate_messages_py)
endif()
