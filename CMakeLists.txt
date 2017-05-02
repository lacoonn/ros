cmake_minimum_required(VERSION 2.8.3)
project(knu_ros_lecture)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_generation
  cv_bridge
  actionlib
  actionlib_msgs
)

add_message_files(
  FILES
  knuRosLecture.msg
)

add_service_files(
	FILES
	srvKnuRosLecture.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

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
add_executable(user_msg_publisher_tutorial src/user_msg_publisher_tutorial.cpp)
add_executable(user_msg_subscriber_tutorial src/user_msg_subscriber_tutorial.cpp)
add_executable(server_srv_tutorial src/server_srv_tutorial.cpp)
add_executable(client_srv_tutorial src/client_srv_tutorial.cpp)
add_executable(server_srv_tutorial_param src/server_srv_tutorial_param.cpp)
add_executable(client_srv_tutorial_param src/client_srv_tutorial_param.cpp)
add_executable(turtle_position_move_new_2017 src/turtle_position_move_new_2017.cpp)
add_executable(turtle_position_move src/turtle_position_move.cpp)



add_dependencies(hello_world_node knu_ros_lecture_generate_message_cpp)
add_dependencies(talker knu_ros_lecture_generate_messages_cpp)
add_dependencies(listeners knu_ros_lecture_generate_message_cpp)
add_dependencies(turtlesim_random_move knu_ros_lecture_generate_messages_cpp)
add_dependencies(turtlesim_pose knu_ros_lecture_generate_messages_cpp)
add_dependencies(turtlesim_cycle knu_ros_lecture_generate_messages_cpp)
add_dependencies(user_msg_publisher_tutorial knu_ros_lecture_generate_messages_cpp)
add_dependencies(user_msg_subscriber_tutorial knu_ros_lecture_generate_messages_cpp)
add_dependencies(server_srv_tutorial knu_ros_lecture_generate_messages_cpp)
add_dependencies(client_srv_tutorial knu_ros_lecture_generate_messages_cpp)
add_dependencies(server_srv_tutorial_param knu_ros_lecture_generate_messages_cpp)
add_dependencies(client_srv_tutorial_param knu_ros_lecture_generate_messages_cpp)
add_dependencies(turtle_position_move_new_2017 knu_ros_lecture_generate_messages_cpp)
add_dependencies(turtle_position_move knu_ros_lecture_generate_messages_cpp)



target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listeners ${catkin_LIBRARIES})
target_link_libraries(hello_world_node ${catkin_LIBRARIES})
target_link_libraries(turtlesim_random_move ${catkin_LIBRARIES})
target_link_libraries(turtlesim_pose ${catkin_LIBRARIES})
target_link_libraries(turtlesim_cycle ${catkin_LIBRARIES})
target_link_libraries(user_msg_publisher_tutorial ${catkin_LIBRARIES})
target_link_libraries(user_msg_subscriber_tutorial ${catkin_LIBRARIES})
target_link_libraries(server_srv_tutorial ${catkin_LIBRARIES})
target_link_libraries(client_srv_tutorial ${catkin_LIBRARIES})
target_link_libraries(server_srv_tutorial_param ${catkin_LIBRARIES} )
target_link_libraries(client_srv_tutorial_param ${catkin_LIBRARIES} )
target_link_libraries(turtle_position_move_new_2017 ${catkin_LIBRARIES} )
target_link_libraries(turtle_position_move ${catkin_LIBRARIES} )


# turtle_position_move_from_file(hw3)
add_executable(turtle_position_move_from_file src/turtle_position_move_from_file.cpp)
add_dependencies(turtle_position_move_from_file knu_ros_lecture_generate_messages_cpp)
target_link_libraries(turtle_position_move_from_file ${catkin_LIBRARIES} )

# opencv
add_executable(opencv_intro src/opencv_intro.cpp)
add_dependencies(opencv_intro knu_ros_lecture_generate_messages_cpp)
target_link_libraries(opencv_intro ${catkin_LIBRARIES} )
