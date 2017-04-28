#include <ros/ros.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define toRadian(degree) ((degree) * (M_PI / 180.))
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose npose;
bool getMsg;

void poseMessageReceived(const nav_msgs::Odometry &msg)
{
  npose = msg.pose.pose;
  //double angle = atan2(2 * pose.orientation.w * pose.orientation.z, 1 - 2 * pow(pose.orientation.z, 2));
  //ROS_INFO_STREAM("position = (" << msg.pose.pose.position.x << "," << msg.pose.pose.position.y << ")");
  getMsg = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

  ros::init(argc, argv, "turtle_position_move_from_file");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/odom", 1000, &poseMessageReceived);

  // open stream input file
  ifstream inStream("/home/turtle/input.txt");

  // exception
  double goal_x, goal_y;
  double now_x, now_y;
  string temp;

  while(1) {
    getMsg = false;
    while(!getMsg) {
      ros::spinOnce();
    }
    inStream >> goal_x >> goal_y;
    now_x = npose.position.x;
    now_y = npose.position.y;

    double translation = sqrt(pow(goal_x - now_x, 2) + pow(goal_y - now_y, 2));
    double rotation = atan2(goal_y - now_y, goal_x - now_x);
    double angle = atan2(2 * npose.orientation.w * npose.orientation.z, 1 - 2 * pow(npose.orientation.z, 2));
    ROS_INFO_STREAM("Now Position : "<< now_x << ", " << now_y);
    ROS_INFO_STREAM("Goal : " << goal_x << ", " << goal_y);
    ROS_INFO_STREAM("Translation : " << translation << ", " << "Rotation : " << rotation - angle);


    //actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
    actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("/cmd_vel_mux/input/teleop");
    client.waitForServer();

    turtlebot_actions::TurtlebotMoveGoal goal;
    //goal.turn_distance = toRadian(rotation);
    goal.turn_distance = rotation - angle;
    goal.forward_distance = translation;
    if(client.sendGoalAndWait(goal, ros::Duration(50.0), ros::Duration(50.0)) == actionlib::SimpleClientGoalState::SUCCEEDED) {
      printf("Call to action server succeeded!\n");
    }
    else {
      printf("Call to action server failed!\n");
    }
    if(inStream.eof())
      break;

  }


  return 0;
}
