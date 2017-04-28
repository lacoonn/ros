#include <ros/ros.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define toRadian(degree) ((degree) * (M_PI / 180.))
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
// ROS를 초기화
ros::init(argc, argv, "turtle_position_move");
// exception
if(argc != 3) {
printf(">> rosrun knu_ros_lecture turtle_position_move [rot_degree] [trans_meter]\n");
return 1;
}
// 파라미터 받아오기
double rotation = atof(argv[1]);
double translation = atof(argv[2]);
// SimpleActionClient 객체 생성
actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
// 서버가 준비될 때까지 대기
client.waitForServer();
// 이동을 지시!
turtlebot_actions::TurtlebotMoveGoal goal;
goal.turn_distance = toRadian(rotation);
goal.forward_distance = translation;
if(client.sendGoalAndWait(goal, ros::Duration(50.0), ros::Duration(50.0)) == actionlib::SimpleClientGoalState::SUCCEEDED) {
printf("Call to action server succeeded!\n");
}
else {
printf("Call to action server failed!\n");
}
return 0;
}
