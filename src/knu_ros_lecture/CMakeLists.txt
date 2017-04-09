cmake_minimum_required(VERSION 2.8.3)
project(knu_ros_lecture)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  knuRosLecture.msg
)

#generate_message(
#	DEPENDENCIES
#	std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES knu_ros_lecture
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hello_world_node src/hello_world.cpp)
add_executable(talker src/talker.cpp)
add_executable(listeners src/listener.cpp)
add_executable(turtlesim_random_move src/turtlesim_random_move.cpp)
add_executable(turtlesim_pose src/turtlesim_pose.cpp)
add_executable(turtlesim_cycle src/turtlesim_cycle.cpp)

add_dependencies(hello_world_node knu_ros_lecture_generate_message_cpp)
add_dependencies(talker knu_ros_lecture_generate_messages_cpp)
add_dependencies(listeners knu_ros_lecture_generate_message_cpp)
add_dependencies(turtlesim_random_move knu_ros_lecture_generate_messages_cpp)
add_dependencies(turtlesim_pose knu_ros_lecture_generate_messages_cpp)
add_dependencies(turtlesim_cycle knu_ros_lecture_generate_messages_cpp)

target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listeners ${catkin_LIBRARIES})
target_link_libraries(hello_world_node ${catkin_LIBRARIES})
target_link_libraries(turtlesim_random_move ${catkin_LIBRARIES})
target_link_libraries(turtlesim_pose ${catkin_LIBRARIES})
target_link_libraries(turtlesim_cycle ${catkin_LIBRARIES})
