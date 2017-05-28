#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>


using namespace cv;
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
#define ROWS 800
#define COLS 800
#define ENLARGEMENT 1



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex[3];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
nav_msgs::OccupancyGrid g_grid;
Mat display;




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A template method to check 'nan'
template<typename T>
inline bool isnan(T value)
{
  return value != value;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void odomMsgCallback(const nav_msgs::Odometry &msg)
{
  // receive a '/odom' message with the mutex
  mutex[0].lock(); {
    g_odom = msg;
  } mutex[0].unlock();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
  // receive a '/odom' message with the mutex
  mutex[1].lock(); {
    g_scan = msg;
  } mutex[1].unlock();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void occupancyGrid_Callback(const nav_msgs::OccupancyGrid grid)
{
  // receive a '/odom' message with the mutex
  mutex[2].lock(); {
    g_grid = grid;
  } mutex[2].unlock();
}

void drawOccupancyGrid(Mat &display)
{
  int width = g_grid.info.width;
  int height = g_grid.info.height;
  int map_origin_x = g_grid.info.origin.position.x;
  int map_origin_y = g_grid.info.origin.position.y;
  double map_origin_angle = atan2(2 * g_grid.info.origin.orientation.w * g_grid.info.origin.orientation.z, 1 - 2 * pow(g_grid.info.origin.orientation.z, 2));
  // mX = X * cos(radian) - Y * sin(radian)
  // mY = X * sin(radian) + Y * cos(radian)
  //int turtlebot_x = (map_origin_x + g_odom.pose.pose.position.x * cos(map_origin_angle) - g_odom.pose.pose.position.y * sin(map_origin_angle)) / g_grid.info.resolution;
  //int turtlebot_y = (map_origin_y + g_odom.pose.pose.position.x * sin(map_origin_angle) + g_odom.pose.pose.position.y * cos(map_origin_angle)) / g_grid.info.resolution;
  // 아래에 것이 유력함
  //int turtlebot_x = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * cos(map_origin_angle) - (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * sin(map_origin_angle);
  //int turtlebot_y = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * sin(map_origin_angle) + (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * cos(map_origin_angle);
  int turtlebot_x = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * sin(map_origin_angle) + (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * cos(map_origin_angle);
  int turtlebot_y = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * cos(map_origin_angle) - (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * sin(map_origin_angle);
  //int turtlebot_x = g_odom.pose.pose.position.x / g_grid.info.resolution;
  //int turtlebot_y = g_odom.pose.pose.position.y / g_grid.info.resolution;
  //int origin_x = COLS/2 + map_origin_x;
  //int origin_y = ROWS/2 + map_origin_y;
  //printf("x : %d, y : %d \n", turtlebot_x, turtlebot_y);

  //Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

  //int nSize = (int) trajectory.size();

  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      int index = width * y + x;
      if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
        if (g_grid.data[index] > 0)
          display.at<Vec3b>(x * ENLARGEMENT, y * ENLARGEMENT) = Vec3b(255, 255, 255);
        else if (g_grid.data[index] == 0)
          display.at<Vec3b>(x * ENLARGEMENT, y * ENLARGEMENT) = Vec3b(127, 127, 127);
      }
    }
  }
  // Current Turtlebot's Position
  circle(display, Point(turtlebot_x * ENLARGEMENT, turtlebot_y * ENLARGEMENT), 5 * ENLARGEMENT, CV_RGB(0, 0, 255), 1, CV_AA);
  // Viewing Direction of Turtlebot
  double turtlebot_angle = atan2(2 * g_odom.pose.pose.orientation.w * g_odom.pose.pose.orientation.z, 1 - 2 * pow(g_odom.pose.pose.orientation.z, 2));
  double map_turtlebot_angle = turtlebot_angle + map_origin_angle;
  // point x = cos(Rad) * length
  // point y = sin(Rad) * length
  int line_length = 5;
  line(display,
   Point(turtlebot_x * ENLARGEMENT, turtlebot_y * ENLARGEMENT),
    Point((turtlebot_x + line_length * sin(map_turtlebot_angle)) * ENLARGEMENT, (turtlebot_y + line_length * cos(map_turtlebot_angle)) * ENLARGEMENT),
     CV_RGB(255, 0, 255), 1, CV_AA);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_map");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 131072, occupancyGrid_Callback);
  ros::Subscriber subOdom = n.subscribe("/odom", 100, &odomMsgCallback);
	
	//initGrid(display, 1600);
	display = Mat::zeros(800, 800, CV_8UC3);
  //display = cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)); // init :: gray image

  while(ros::ok()){
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    display.setTo(Scalar(0, 0, 0));
    drawOccupancyGrid(display);
    cv::imshow("Map", display);
    cv::waitKey(30);
  }

  return 0;
}
