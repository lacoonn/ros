#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iomanip>

#define DIS 0.1
#define SCOPE 0.005
#define SPEED 1
#define ROTATE 0.10

// 거북은 이동과 회전 상태를 반복하며 한붓그리기를 실행합니다.
enum state {ST1, RT1, ST2, RT2, ST3, RT3, ST4, RT4, ST5, RT5, ST6, RT6, ST7,
  RT7, ST8, END};

// 거북이 이동 상태에서 목표 위치에 도달했는지 확인하는 함수입니다.
// 정확한 위치로 이동하기 힘들기 때문에 범위를 정해 그 안에 도달하면 도착으로 인정합니다.
bool isArrive(double a, double b, double x, double y)
{
    if ((a-x)*(a-x) + (b-y)*(b-y) < DIS * DIS)
        return true;
    return false;
}

// publish와 subscribe를 함께 하기 위해 클래스를 사용했습니다.
class turtlesim_cycle
{
public:
    turtlesim_cycle()
    {
        pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
        sub = nh.subscribe("turtle1/pose", 100, &turtlesim_cycle::poseMessageReceived, this);
        nstate = ST1;
    }
    //CallBack 함수입니다.
    void poseMessageReceived(const turtlesim::Pose &mssg)
    {
        msg = mssg;

        ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
            "position = (" << mssg.x << "," << mssg.y << ")" << " direction = " << mssg.theta << " state = " << nstate);
        //subscribe해서 위치를 파악할 때마다 함수를 실행해 상태를 업데이트합니다.
        publishCycle();
    }

    void publishCycle() {
      // 상태별로 이동해가면서 최종적으로 한붓그리기를 실행합니다.
        switch(nstate) {
        case ST1: // ST# 상태는 이동을 위한 상태입니다. 이동은 x축으로만 하고 목표 위치에 도달하면 RT# 상태로 이동합니다.
            move.linear.x = 1;
            move.angular.z = 0;
            if(isArrive(10, 5.54, msg.x, msg.y))
              nstate = RT1;
            break;
        case RT1: // RT# 상태는 회전을 위한 상태입니다. 회전하다가 목표 각에 도달하면 다음 상태로 이동합니다.
            move.linear.x = 0;
          	move.angular.z = ROTATE;
          	if(1.570 - SCOPE < msg.theta && msg.theta < 1.570 + SCOPE)
          		nstate = ST2;
            break;
        case ST2:
            move.linear.x = 1;
            move.angular.z = 0;
            if(isArrive(10, 8, msg.x, msg.y))
              nstate = RT2;
            break;
        case RT2:
          	move.linear.x = 0;
          	move.angular.z = ROTATE;
          	if(2.356 - SCOPE < msg.theta && msg.theta < 2.356 + SCOPE)
          		nstate = ST3;
            break;
        case ST3:
            move.linear.x = 1;
            move.angular.z = 0;
            if(isArrive(8, 10, msg.x, msg.y))
              nstate = RT3;
            break;
        case RT3:
            move.linear.x = 0;
          	move.angular.z = ROTATE;
            if(3.926 - SCOPE < msg.theta && msg.theta < 3.926 + SCOPE)
          		nstate = ST4;
            break;
        case ST4:
            move.linear.x = SPEED;
            move.angular.z = 0;
            if(isArrive(6, 8, msg.x, msg.y))
              nstate = RT4;
            break;
        case RT4:
            move.linear.x = 0;
          	move.angular.z = ROTATE;
            if(4.71 - SCOPE < msg.theta && msg.theta < 4.71 + SCOPE)
          		nstate = ST5;
            break;
        case ST5:
            move.linear.x = SPEED;
            move.angular.z = 0;
            if(isArrive(6, 5.54, msg.x, msg.y))
              nstate = RT5;
            break;
        case RT5:
            move.linear.x = 0;
          	move.angular.z = ROTATE;
            if(0.558 < msg.theta && msg.theta < 0.558 + SCOPE)
          		nstate = ST6;
            break;
        case ST6:
            move.linear.x = SPEED;
            move.angular.z = 0;
            if(isArrive(10, 8, msg.x, msg.y))
              nstate = RT6;
            break;
        case RT6:
            move.linear.x = 0;
          	move.angular.z = ROTATE;
            if(3.141 < msg.theta && msg.theta < 3.141 + SCOPE)
          		nstate = ST7;
            break;
        case ST7:
            move.linear.x = SPEED;
            move.angular.z = 0;
            if(isArrive(6, 8, msg.x, msg.y))
              nstate = RT7;
            break;
        case RT7:
            move.linear.x = 0;
          	move.angular.z = ROTATE;
            if(5.72 < msg.theta && msg.theta < 5.72 + SCOPE)
          		nstate = ST8;
            break;
        case ST8:
            move.linear.x = SPEED;
            move.angular.z = 0;
            if(isArrive(10, 5.54, msg.x, msg.y))
              nstate = END;
            break;
        case END:
            move.linear.x = 0;
            move.angular.z = 0;
    }
    pub.publish(move); // 현재 상태에서 가져야할 벡터를 publish합니다.
    ROS_INFO_STREAM("now velocity command:" << " linear=" << move.linear.x <<
        " angular=" << move.angular.z << " state = " << nstate);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::Twist move;
    enum state nstate;
    turtlesim::Pose msg;


};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesim_cycle");
    turtlesim_cycle turtleCycle;

    ros::spin();

    return 0;
}
