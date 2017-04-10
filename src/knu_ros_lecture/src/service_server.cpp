#include "ros/ros.h"
   2 #include "knu_ros_lecture/AddTwoInts.h"
   3
   4 bool add(knu_ros_lecture::AddTwoInts::Request  &req,
   5          knu_ros_lecture::AddTwoInts::Response &res)
   6 {
   7   res.sum = req.a + req.b;
   8   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
   9   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  10   return true;
  11 }
  12
  13 int main(int argc, char **argv)
  14 {
  15   ros::init(argc, argv, "add_two_ints_server");
  16   ros::NodeHandle n;
  17
  18   ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  19   ROS_INFO("Ready to add two ints.");
  20   ros::spin();
  21
  22   return 0;
  23 }
