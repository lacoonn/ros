# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "knu_ros_lecture: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iknu_ros_lecture:/home/turtle/catkin_ws/src/knu_ros_lecture/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(knu_ros_lecture_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" NAME_WE)
add_custom_target(_knu_ros_lecture_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "knu_ros_lecture" "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" ""
)

get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" NAME_WE)
add_custom_target(_knu_ros_lecture_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "knu_ros_lecture" "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/knu_ros_lecture
)

### Generating Services
_generate_srv_cpp(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/knu_ros_lecture
)

### Generating Module File
_generate_module_cpp(knu_ros_lecture
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/knu_ros_lecture
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(knu_ros_lecture_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(knu_ros_lecture_generate_messages knu_ros_lecture_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_cpp _knu_ros_lecture_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_cpp _knu_ros_lecture_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(knu_ros_lecture_gencpp)
add_dependencies(knu_ros_lecture_gencpp knu_ros_lecture_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS knu_ros_lecture_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/knu_ros_lecture
)

### Generating Services
_generate_srv_eus(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/knu_ros_lecture
)

### Generating Module File
_generate_module_eus(knu_ros_lecture
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/knu_ros_lecture
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(knu_ros_lecture_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(knu_ros_lecture_generate_messages knu_ros_lecture_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_eus _knu_ros_lecture_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_eus _knu_ros_lecture_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(knu_ros_lecture_geneus)
add_dependencies(knu_ros_lecture_geneus knu_ros_lecture_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS knu_ros_lecture_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/knu_ros_lecture
)

### Generating Services
_generate_srv_lisp(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/knu_ros_lecture
)

### Generating Module File
_generate_module_lisp(knu_ros_lecture
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/knu_ros_lecture
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(knu_ros_lecture_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(knu_ros_lecture_generate_messages knu_ros_lecture_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_lisp _knu_ros_lecture_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_lisp _knu_ros_lecture_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(knu_ros_lecture_genlisp)
add_dependencies(knu_ros_lecture_genlisp knu_ros_lecture_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS knu_ros_lecture_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/knu_ros_lecture
)

### Generating Services
_generate_srv_nodejs(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/knu_ros_lecture
)

### Generating Module File
_generate_module_nodejs(knu_ros_lecture
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/knu_ros_lecture
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(knu_ros_lecture_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(knu_ros_lecture_generate_messages knu_ros_lecture_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_nodejs _knu_ros_lecture_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_nodejs _knu_ros_lecture_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(knu_ros_lecture_gennodejs)
add_dependencies(knu_ros_lecture_gennodejs knu_ros_lecture_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS knu_ros_lecture_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/knu_ros_lecture
)

### Generating Services
_generate_srv_py(knu_ros_lecture
  "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/knu_ros_lecture
)

### Generating Module File
_generate_module_py(knu_ros_lecture
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/knu_ros_lecture
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(knu_ros_lecture_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(knu_ros_lecture_generate_messages knu_ros_lecture_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/msg/knuRosLecture.msg" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_py _knu_ros_lecture_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtle/catkin_ws/src/knu_ros_lecture/srv/srvKnuRosLecture.srv" NAME_WE)
add_dependencies(knu_ros_lecture_generate_messages_py _knu_ros_lecture_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(knu_ros_lecture_genpy)
add_dependencies(knu_ros_lecture_genpy knu_ros_lecture_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS knu_ros_lecture_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/knu_ros_lecture)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/knu_ros_lecture
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(knu_ros_lecture_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/knu_ros_lecture)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/knu_ros_lecture
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(knu_ros_lecture_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/knu_ros_lecture)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/knu_ros_lecture
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(knu_ros_lecture_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/knu_ros_lecture)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/knu_ros_lecture
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(knu_ros_lecture_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/knu_ros_lecture)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/knu_ros_lecture\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/knu_ros_lecture
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(knu_ros_lecture_generate_messages_py std_msgs_generate_messages_py)
endif()
