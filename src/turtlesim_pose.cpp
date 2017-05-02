#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iomanip>


void poseMessageReceived(const nav_msgs::Odometry &msg)
{
  geometry_msgs::Pose pose = msg.pose.pose;
  geometry_msgs::Twist twist = msg.twist.twist;
  double angle = atan2(2 * pose.orientation.w * pose.orientation.z, 1 - 2 * pow(pose.orientation.z, 2));
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
  "position = (" << pose.position.x << "," << pose.position.y << ")" << " direction = " << angle);


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlesim_pose");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/odom", 1000, &poseMessageReceived);

  ros::spin();

  return 0;
}
