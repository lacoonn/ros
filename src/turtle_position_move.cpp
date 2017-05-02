#include <ros/ros.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define toRadian(degree) ((degree) * (M_PI / 180.))
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
// ROS
ros::init(argc, argv, "turtle_position_move");
// exception
if(argc != 3) {
printf(">> rosrun knu_ros_lecture turtle_position_move [rot_degree] [trans_meter]\n");
return 1;
}
//
double rotation = atof(argv[1]);
double translation = atof(argv[2]);
// SimpleActionClient 
actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
//
client.waitForServer();
//
turtlebot_actions::TurtlebotMoveGoal goal;
cout << "radian : " << toRadian(rotation) << "\n";
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
