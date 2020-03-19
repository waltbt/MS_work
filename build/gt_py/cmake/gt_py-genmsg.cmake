# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gt_py: 3 messages, 0 services")

set(MSG_I_FLAGS "-Igt_py:/home/ben/gripper_proj_py/src/gt_py/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gt_py_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" NAME_WE)
add_custom_target(_gt_py_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gt_py" "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" ""
)

get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" NAME_WE)
add_custom_target(_gt_py_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gt_py" "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" ""
)

get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" NAME_WE)
add_custom_target(_gt_py_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gt_py" "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gt_py
)
_generate_msg_cpp(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gt_py
)
_generate_msg_cpp(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gt_py
)

### Generating Services

### Generating Module File
_generate_module_cpp(gt_py
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gt_py
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gt_py_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gt_py_generate_messages gt_py_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_cpp _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_cpp _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_cpp _gt_py_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gt_py_gencpp)
add_dependencies(gt_py_gencpp gt_py_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gt_py_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gt_py
)
_generate_msg_eus(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gt_py
)
_generate_msg_eus(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gt_py
)

### Generating Services

### Generating Module File
_generate_module_eus(gt_py
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gt_py
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gt_py_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gt_py_generate_messages gt_py_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_eus _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_eus _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_eus _gt_py_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gt_py_geneus)
add_dependencies(gt_py_geneus gt_py_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gt_py_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gt_py
)
_generate_msg_lisp(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gt_py
)
_generate_msg_lisp(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gt_py
)

### Generating Services

### Generating Module File
_generate_module_lisp(gt_py
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gt_py
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gt_py_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gt_py_generate_messages gt_py_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_lisp _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_lisp _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_lisp _gt_py_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gt_py_genlisp)
add_dependencies(gt_py_genlisp gt_py_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gt_py_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gt_py
)
_generate_msg_nodejs(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gt_py
)
_generate_msg_nodejs(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gt_py
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gt_py
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gt_py
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gt_py_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gt_py_generate_messages gt_py_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_nodejs _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_nodejs _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_nodejs _gt_py_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gt_py_gennodejs)
add_dependencies(gt_py_gennodejs gt_py_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gt_py_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py
)
_generate_msg_py(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py
)
_generate_msg_py(gt_py
  "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py
)

### Generating Services

### Generating Module File
_generate_module_py(gt_py
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gt_py_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gt_py_generate_messages gt_py_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/color_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_py _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/IR_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_py _gt_py_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ben/gripper_proj_py/src/gt_py/msg/TOF_sensor_data.msg" NAME_WE)
add_dependencies(gt_py_generate_messages_py _gt_py_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gt_py_genpy)
add_dependencies(gt_py_genpy gt_py_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gt_py_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gt_py)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gt_py
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gt_py_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gt_py)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gt_py
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gt_py_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gt_py)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gt_py
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gt_py_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gt_py)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gt_py
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gt_py_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gt_py
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gt_py_generate_messages_py std_msgs_generate_messages_py)
endif()
