#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
double angle( Point pt1, Point pt2, Point pt0 ) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
void find_squares( Mat& image, vector< vector< Point> >& squares)
{
  // blur will enhance edge detection
  GaussianBlur(image, image, Size(5,5), 1 ,1);
  Mat gray0;
  cvtColor(image,gray0, CV_BGR2GRAY);
  Mat gray(image.size(), CV_8U);
  vector< vector< Point> > contours;
  gray = gray0 >= 100;
  // Find contours and store them in a list
  findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  // Test contours
  vector< Point> approx;
  for (size_t i = 0; i < contours.size(); i++)
  {
    // approximate contour with accuracy proportional
    // to the contour perimeter
    approxPolyDP( Mat(contours[i]), approx, arcLength( Mat(contours[i]), true)*0.02, true);
    // Note: absolute value of an area is used because
    // area may be positive or negative - in accordance with the
    // contour orientation
    if (approx.size() == 4 &&
    fabs(contourArea( Mat(approx))) > 1000 &&
    isContourConvex( Mat(approx)))
    {
      double maxCosine = 0;
      for (int j = 2; j < 5; j++)
      {
        double cosine = fabs(angle(approx[ j%4], approx[ j-2], approx[ j-1]));
        maxCosine = MAX(maxCosine, cosine);
      }
      if (maxCosine < 0.3)
      squares.push_back(approx);
    }
  }
}

void find_largest_square(const vector<vector <Point> >& squares, vector<Point>& biggest_square) {
  if (!squares.size()) {
    return;
  }
  int max_width = 0;
  int max_height = 0;
  int max_square_idx = 0;
  const int n_points = 4;
  for (size_t i = 0; i < squares.size(); i++) {
    Rect rectangle = boundingRect(Mat(squares[i]));
    if ((rectangle.width >= max_width) && (rectangle.height >= max_height)) {
      max_width = rectangle.width;
      max_height = rectangle.height;
      max_square_idx = i;
    }
  }
  biggest_square = squares[max_square_idx];
}
// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedRGB(const sensor_msgs::Image& msg) {
  ROS_INFO("seq = %d / width = %d / height = %d / step = %d", msg.header.seq, msg.width, msg.height,
  msg.step);
  ROS_INFO("encoding = %s", msg.encoding.c_str());
  Mat img = Mat(msg.height, msg.width, CV_8UC3);
  memcpy(img.data, &msg.data[0], sizeof(unsigned char)*msg.data.size());
  Mat img_origin = img.clone();
  vector< vector< Point> > squares;
  find_squares(img, squares);
  vector<Point> largest_square;
  find_largest_square(squares, largest_square);
  if(largest_square.size() >0 ) {
    for (int i = 0; i < 4; i++) {
      line(img, largest_square[i], largest_square[(i+1)%4], Scalar(0, 0, 255), 3, CV_AA);
    }
  }
  //for (int i = 0; i < squres.size(); i++)
  //for (int j = 0; j < 4; j++)
  // line(img, square[i][ j], square[i][( j+1)%4], Scalar(0, 0, 255), 3, CV_AA);
  imshow("img_origin",img_origin);
  imshow("squares", img);
  waitKey(30);
}
int main(int argc, char **argv)
{
  // Initialize the ROS system
  ros::init(argc, argv, "turtle_kinect_image_view");
  ros::NodeHandle nh;
  // Create a subscriber object
  ros::Subscriber subRgb = nh.subscribe("/camera/rgb/image_color", 10, &poseMessageReceivedRGB);
  // Let ROS take over
  ros::spin();
  return 0;
}
