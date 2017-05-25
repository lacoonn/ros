#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A template method to check 'nan'
template<typename T>
inline bool isnan(T value)
{
    return value != value;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex[0].lock(); {
        g_odom = msg;
    } mutex[0].unlock();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    // receive a '/odom' message with the mutex
    mutex[1].lock(); {
        g_scan = msg;
    } mutex[1].unlock();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
    // �̵� ����
    xyz[0] = odom.pose.pose.position.x;
    xyz[1] = odom.pose.pose.position.y;
    xyz[2] = odom.pose.pose.position.z;

    // ȸ�� ����
    tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();
    XYZs.resize(nRangeSize);

    for(int i=0; i<nRangeSize; i++) {
        double dRange = lrfScan.ranges[i];

        if(isnan(dRange)) {
            XYZs[i] = Vec3d(0., 0., 0.);
        } else {
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist)
{
    int nSize = (int) trajectory.size();

    if(nSize <= 0) {
        trajectory.push_back(xyz);
    } else {
        Vec3d diff = trajectory[nSize-1] - xyz;
        double len = sqrt(diff.dot(diff));

        if(len > dMinDist) {
            trajectory.push_back(xyz);
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
transform(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
    Vec3d newPt;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++) {
        newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
        newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
        newPt[2];
        laserScanXY[i] = newPt;
    }
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
initGrid(Mat &display, int nImageSize)
{
    const int nImageHalfSize = nImageSize/2;
    const int nAxisSize = nImageSize/16;
    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
drawTrajectory(Mat &display, vector<Vec3d> &trajectory, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

    int nSize = (int) trajectory.size();

    for(int i=0; i<nSize; i++) {
        int x = imageHalfSize[0] + cvRound((trajectory[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((trajectory[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
            display.at<Vec3b>(y, x) = Vec3b(0, 255, 255);
            //circle(display, Point(x, y), 1, CV_RGB(255, 255, 0), 2, CV_AA);
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
drawCurrentPositionWithRotation(Mat &display, Vec3d &xyz, Vec3d &rpy, double dMaxDist)
{
    printf("_r = %.3lf, _p = %.3lf, _y = %.3lf\n", toDegree(rpy[0]), toDegree(rpy[1]), toDegree(rpy[2]));

    const int nHeadingSize = 30;
    Vec2i headingDir = Vec2i(nHeadingSize*cos(rpy[2]), nHeadingSize*sin(rpy[2]));
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

    int x = imageHalfSize[0] + cvRound((xyz[0]/dMaxDist)*imageHalfSize[0]);
    int y = imageHalfSize[1] + cvRound((xyz[1]/dMaxDist)*imageHalfSize[1]);

    if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
        circle(display, Point(x, y), nHeadingSize, CV_RGB(255, 0, 255), 1, CV_AA);
        line(display, Point(x, y), Point(x+headingDir[0], y+headingDir[1]), CV_RGB(255, 0, 255), 1, CV_AA);
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++) {
        int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
            display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void
drawLRFScanMulti(Mat &display, vector< vector<Vec3d> > &laserScanXYMulti, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nNumOfScan = (int)laserScanXYMulti.size();

    for(int i=0; i<nNumOfScan; i++) {
        int nRangeSize = (int)laserScanXYMulti[i].size();

        for(int j=0; j<nRangeSize; j++) {
            int x = imageHalfSize[0] + cvRound((laserScanXYMulti[i][j][0]/dMaxDist)*imageHalfSize[0]);
            int y = imageHalfSize[1] + cvRound((laserScanXYMulti[i][j][1]/dMaxDist)*imageHalfSize[1]);

            if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
                display.at<Vec3b>(y, x) = Vec3b(128, 128, 128);
            }
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
printOdometryInfo(nav_msgs::Odometry &odom)
{
    // Display /odom part!
    const ros::Time timestamp = odom.header.stamp;
    const string frame_id = odom.header.frame_id;
    const string child_frame_id = odom.child_frame_id;
    const geometry_msgs::Point translation = odom.pose.pose.position;
    const geometry_msgs::Quaternion rotation = odom.pose.pose.orientation;

    printf("frame_id = %s, child_frame_id = %s\n", frame_id.c_str(), child_frame_id.c_str());
    printf("secs: %d / nsecs: %d\n", timestamp.sec, timestamp.nsec);
    printf("translation = %lf %lf %lf\n", translation.x, translation.y, translation.z);
    printf("rotation = %lf %lf %lf %lf\n\n\n", rotation.x, rotation.y, rotation.z, rotation.w);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtle_position_lrf_view");
    ros::NodeHandle nh;

    // Create subscriber objects
    ros::Subscriber subOdom = nh.subscribe("/odom", 100, &odomMsgCallback);
    ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);

    // Display buffer
    Mat display;
    initGrid(display, 801);

    // Odometry buffer
    nav_msgs::Odometry odom;

    // Scan buffer
    sensor_msgs::LaserScan scan;

    // �̵� �� ȸ�� ����
    Vec3d xyz, rpy;

    // �̵�����
    vector<Vec3d> trajectory;

    // LRF scan ����
    vector<Vec3d> laserScanXY;
    vector< vector<Vec3d> > laserScanXYMulti;

    // Mat distance for grid
    const double dGridMaxDist = 4.0;

    // main loop
    while(ros::ok()) {
        // callback �Լ��� call!
        ros::spinOnce();

        // receive the global '/odom' message with the mutex
        mutex[0].lock(); {
           odom = g_odom;
        } mutex[0].unlock();

        // odom���κ��� �̵� �� ȸ������ ȹ��
        convertOdom2XYZRPY(odom, xyz, rpy);

        // ������ ��ġ�� ����
        saveCurrentPosition(xyz, trajectory, 0.02);

        // receive the global '/scan' message with the mutex
        mutex[1].lock(); {
           scan = g_scan;
        } mutex[1].unlock();

        // scan���κ��� Cartesian X-Y scan ȹ��
        convertScan2XYZs(scan, laserScanXY);

        // laserScan�� ������ǥ��� ��ȯ
        transform(laserScanXY, xyz[0], xyz[1], rpy[2]);

        // ���� ��Ȳ�� draw�� display �̹����� ����
        initGrid(display, 801);
        drawTrajectory(display, trajectory, dGridMaxDist);
        drawCurrentPositionWithRotation(display, xyz, rpy, dGridMaxDist);
        drawLRFScanMulti(display, laserScanXYMulti, dGridMaxDist);
        drawLRFScan(display, laserScanXY, dGridMaxDist);

        laserScanXYMulti.push_back(laserScanXY);

        // 2D ������ǥ�迡�� top-view ����� 3���� ������ǥ��� ��ȯ
        transpose(display, display);  // X-Y�� ��ȯ
        flip(display, display, 0);  // ������� ����
        flip(display, display, 1);  // �������� ����

        // ���� ���!
        imshow("KNU ROS Lecture >> turtle_position_lrf_view", display);
        printOdometryInfo(odom);

        // ������� Ű���� �Է��� ����!
        int nKey = waitKey(30) % 255;

        if(nKey == 27) {
            // ����
            break;
        } else if(nKey == ' ') {
            laserScanXYMulti.push_back(laserScanXY);
        } else if(nKey == 'c' || nKey == 'C') {
            initGrid(display, 801);
            laserScanXYMulti.clear();
            trajectory.clear();
        }
    }

    return 0;
}
