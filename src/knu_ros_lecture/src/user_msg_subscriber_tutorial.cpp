#include <ros/ros.h>
#include <knu_ros_lecture/knuRosLecture.h>

void msgCallback(const knu_ros_-lecture::knuRosLecture &msg)
{
  ROS_INFO("Received Msg : %s / %.2lf / %.f / %d", msg.stringData.c_star(),
  msg.float64Data, msg.float32Data, msg.int32Data);
}

int main(int argc, char **argv)
{


  return 0;
}
