#include <ros/ros.h>
#include <knu_ros_lecture/knuRosLecture.h>

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "user_msg_publisher_tutorial");
    ros::NodeHandle nh;

    // Decleation of publisher
    ros::Publisher pub = nh.advertise<knu_ros_lecture::knuRosLecture>("user_msg_tutorial", 100);

    // roop_rate set!
    ros::Rate rate(10);
    int count = 0;

    // main loop
    while(ros::ok()) {
        knu_ros_lecture::knuRosLecture msg;

        msg.int32Data = count;
        msg.float32Data = float(count+1);
        msg.float64Data = double(count+2);
        msg.stringData = "crvl";

        // Publish the message.
        pub.publish(msg);

        // Send a message to rosout with thedetails.
        ROS_INFO("send msg = %d / %.2f / %.2lf / %s", msg.int32Data, msg.float32Data, msg.float64Data, msg.stringData.c_str());

        // Wait until it's time for another iteration
        rate.sleep();

        count += 10;
    }

    return 0;
}
