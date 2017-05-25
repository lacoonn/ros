#include <ros/ros.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
// For OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// pthread
#include <pthread.h>

using namespace std;
using namespace cv;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define toRadian(degree) ((degree) * (M_PI / 180.))
#define toDegree(radian) ((radian) * (180. / M_PI))
#define COLS 480
#define ROWS 480
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose npose;
bool getMsg;
bool isFin = false;
// init buffer image
Mat display = Mat::zeros(ROWS, COLS, CV_8UC3);
int portion = 20;

void poseMessageReceived(const nav_msgs::Odometry &msg)
{
	npose = msg.pose.pose;
	getMsg = true;
}

void *printCV(void *data)
{
	/*
	// Chang a pixel value
	for(int r=0; r<50; r++) {
		for(int c=0; c<70; c++) {
			display.at<Vec3b>(100+r, 200+c) = Vec3b(255, 255, 0);
		}
	}
	*/

	// draw image
	while(1) {
		imshow("turtle_position_move_from_file_cv", display);
		int nKey = waitKey(30) % 255;
		if (isFin)
			break;
	}
}


int main(int argc, char **argv)
{

  	ros::init(argc, argv, "turtle_position_move_from_file");
  	ros::NodeHandle nh;
  	ros::Subscriber sub = nh.subscribe("/odom", 1000, &poseMessageReceived);


  	// open input file stream
  	ifstream inStream("/home/turtle/input.txt");

	// pthread for print image
	pthread_t printcv;

	pthread_create(&printcv, NULL, printCV, NULL);

  	// init variable before loop
  	double goal_x, goal_y;
  	double now_x, now_y;
  	double prev_x, prev_y;
  	int loop_count = 0;
  	string temp;

	while(1) {
	    getMsg = false;
	    while(!getMsg) {
		  ros::spinOnce();
    	}
		inStream >> goal_x >> goal_y;
		if(inStream.eof())
			break;
		now_x = npose.position.x;
		now_y = npose.position.y;

		// draw a circle
		circle(display, Point(COLS / 2 + now_x * portion, ROWS / 2 + now_y * portion), 10, CV_RGB(255, 0, 0), 3, CV_AA);

		// draw a line
		if (loop_count++ != 0)
			line(display, Point(COLS / 2 + prev_x * portion, ROWS / 2 + prev_y * portion), Point(COLS / 2 + now_x * portion, ROWS / 2 + now_y * portion), CV_RGB(0, 0, 255), 2, CV_AA);



		double translation = sqrt(pow(goal_x - now_x, 2) + pow(goal_y - now_y, 2));
		double rotation = atan2(goal_y - now_y, goal_x - now_x);
		double angle = atan2(2 * npose.orientation.w * npose.orientation.z, 1 - 2 * pow(npose.orientation.z, 2));



	/*
		//actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
		actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
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
	*/

		// ê°ë„ê°? ë§žì„?•Œê¹Œì?? ?˜¤ì°¨êµ? •
		double count = 1;
		while(abs(angle - rotation) > 0.02) {
			actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
			turtlebot_actions::TurtlebotMoveGoal goal;
			client.waitForServer();
			goal.turn_distance = (rotation - angle) + 5.76;
			cout << "ì§?ê¸ˆê°?„ : " << toDegree(angle) << ", ëª©í‘œê°ë„ : " << toDegree(rotation) << ", ?šŒ? „ê°ë„ : " << toDegree((rotation - angle) + 5.76) << endl;
			if (count < 15);
				count += 1;
			client.sendGoalAndWait(goal, ros::Duration(50.0), ros::Duration(50.0));
			ros::Duration(1).sleep();
			// get new odom information
			getMsg = false;
			while(!getMsg) {
					ros::spinOnce();
			}
	  		angle = atan2(2 * npose.orientation.w * npose.orientation.z, 1 - 2 * pow(npose.orientation.z, 2));
	   	}
	   	// print current information
	   	cout << "Now Position : "<< now_x << ", " << now_y << endl;
		cout << "Goal : " << goal_x << ", " << goal_y << endl;
		cout << "Translation : " << translation << ", Rotation : " << toDegree(rotation) << ", Should_Rotate : " << toDegree(rotation - angle) << endl;
		// move distance
	   	actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> client("turtlebot_move");
		turtlebot_actions::TurtlebotMoveGoal goal;
		client.waitForServer();
		goal.turn_distance = 0;
		goal.forward_distance = translation;
		if(client.sendGoalAndWait(goal, ros::Duration(50.0), ros::Duration(50.0)) == actionlib::SimpleClientGoalState::SUCCEEDED) {
	  		printf("Call to action server succeeded!\n");
		}
		else {
	  		printf("Call to action server failed!\n");
		}

		prev_x = now_x;
		prev_y = now_y;
	}

	isFin = true;
	pthread_join(printcv, NULL);

	return 0;
}
