#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>


using namespace cv;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define ToRadian(degree)	((degree) * (M_PI / 180.))
#define ToDegree(radian)	((radian) * (180. / M_PI))



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
    // 이동 저장
    xyz[0] = odom.pose.pose.position.x;
    xyz[1] = odom.pose.pose.position.y;
    xyz[2] = odom.pose.pose.position.z;

    // 회전 저장
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
        newPt[2] = 0;
        laserScanXY[i] = newPt;
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
transform(vector<Vec3d> &laserScanXY, vector<Vec3d> &laserScanXYAfter, double x, double y, double theta)
{
    Vec3d newPt;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    int nRangeSize = (int)laserScanXY.size();
    laserScanXYAfter.resize(nRangeSize);

    for(int i=0; i<nRangeSize; i++) {
        newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
        newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
        newPt[2] = 0;
        laserScanXYAfter[i] = newPt;
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
    //printf("_r = %.3lf, _p = %.3lf, _y = %.3lf\n", ToDegree(rpy[0]), ToDegree(rpy[1]), ToDegree(rpy[2]));

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
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist, Scalar color)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++) {
        int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
            display.at<Vec3b>(y, x) = Vec3b(color[0], color[1], color[2]);
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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
PrintMatrix(char *pMessage, cv::Mat &mat)
{
    printf("%s\n", pMessage);

    if(mat.type() == CV_32S) {
        for(int r=0; r<mat.rows; r++) {
            for(int c=0; c<mat.cols; c++) {
                printf("%d ", mat.at<int>(r,c));
            } printf("\n");
        } printf("\n");
    } else if(mat.type() == CV_32F) {
        for(int r=0; r<mat.rows; r++) {
            for(int c=0; c<mat.cols; c++) {
                printf("%.8f ", mat.at<float>(r,c));
            } printf("\n");
        } printf("\n");
    } else if(mat.type() == CV_64F) {
        for(int r=0; r<mat.rows; r++) {
            for(int c=0; c<mat.cols; c++) {
                printf("%.8lf ", mat.at<double>(r,c));
            } printf("\n");
        } printf("\n");
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
Mat
EulerToMatrixCCW(double dRx, double dRy, double dRz, bool bIsDegree)
{
    // RPY Euler angle (roll->pitch->yaw) 순서를 따르는 회전 행렬
    // roll = z축 회전
    // pitch = y축 회전
    // yaw = x축 회전
    Mat rx(3, 3, CV_64F), ry(3, 3, CV_64F), rz(3, 3, CV_64F);

    setIdentity(rx);
    setIdentity(ry);
    setIdentity(rz);

    // convert to radian
    if(bIsDegree) {
        dRx = ToRadian(dRx);	dRy = ToRadian(dRy);	dRz = ToRadian(dRz);
    }

    //////////////////////////////////////////////////////////
    // Yaw, make a X-axis rotation matrix
    rx.at<double>(1, 1) = cos(dRx);
    rx.at<double>(1, 2) = -sin(dRx);
    rx.at<double>(2, 1) = -rx.at<double>(1, 2);
    rx.at<double>(2, 2) = rx.at<double>(1, 1);

    //////////////////////////////////////////////////////////
    // Pitch, make a Y-axis rotation matrix
    ry.at<double>(0, 0) = cos(dRy);
    ry.at<double>(0, 2) = sin(dRy);
    ry.at<double>(2, 0) = -ry.at<double>(0, 2);
    ry.at<double>(2, 2) = ry.at<double>(0, 0);

    //////////////////////////////////////////////////////////
    // Roll, make a Z-axis rotation matrix
    rz.at<double>(0, 0) = cos(dRz);
    rz.at<double>(1, 1) = rz.at<double>(0, 0);
    rz.at<double>(0, 1) = -sin(dRz);
    rz.at<double>(1, 0) = -rz.at<double>(0, 1);

    // RPY순으로 곱하기
    Mat ret = rz*ry*rx;

    return ret;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Matrix -> Euler
void
MatrixToEulerCCW(cv::Mat &mat, double &dRx, double &dRy, double &dRz, bool bIsDegree)
{
    // RPY Euler angle (roll->pitch->yaw) 순서를 따르는 회전 행렬
    // roll = z축 회전
    // pitch = y축 회전
    // yaw = x축 회전
    const double dRoll = atan2(mat.at<double>(1,0), mat.at<double>(0,0));
    const double dPitch = atan2(-mat.at<double>(2,0), sqrt(mat.at<double>(2,1)*mat.at<double>(2,1) + mat.at<double>(2,2)*mat.at<double>(2,2)));
    const double dYaw = atan2(mat.at<double>(2,1), mat.at<double>(2,2));

    // convert to radian
    if(bIsDegree) {
        dRx = ToDegree(dYaw);	dRy = ToDegree(dPitch);		dRz = ToDegree(dRoll);
    } else {
        dRx  = dYaw;			dRy  = dPitch;				dRz  = dRoll;
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
Mat
getRelativeRt(nav_msgs::Odometry &odomPrev, nav_msgs::Odometry &odomCrnt)
{
    // 변수선언
    Mat prevWorldRt = Mat::eye(4, 4, CV_64F);
    Mat crntWorldRt = Mat::eye(4, 4, CV_64F);

    // 회전저장
    Vec3d prevRotationEuler, crntRotationEuler;
    tf::Quaternion prevRotationQuat = tf::Quaternion(odomPrev.pose.pose.orientation.x, odomPrev.pose.pose.orientation.y, odomPrev.pose.pose.orientation.z, odomPrev.pose.pose.orientation.w);
    tf::Quaternion crntRotationQuat = tf::Quaternion(odomCrnt.pose.pose.orientation.x, odomCrnt.pose.pose.orientation.y, odomCrnt.pose.pose.orientation.z, odomCrnt.pose.pose.orientation.w);
    tf::Matrix3x3(prevRotationQuat).getEulerYPR(prevRotationEuler[0], prevRotationEuler[1], prevRotationEuler[2]);
    tf::Matrix3x3(crntRotationQuat).getEulerYPR(crntRotationEuler[0], crntRotationEuler[1], crntRotationEuler[2]);

    Mat prevR = EulerToMatrixCCW(prevRotationEuler[2], prevRotationEuler[1], prevRotationEuler[0], false);
    Mat crntR = EulerToMatrixCCW(crntRotationEuler[2], crntRotationEuler[1], crntRotationEuler[0], false);

    for(int r=0; r<3; r++) {
        for(int c=0; c<3; c++) {
            prevWorldRt.at<double>(r, c) = prevR.at<double>(r, c);
            crntWorldRt.at<double>(r, c) = crntR.at<double>(r, c);
        }
    }

    // 위치저장
    Vec3d prevPosition = Vec3d(odomPrev.pose.pose.position.x, odomPrev.pose.pose.position.y, odomPrev.pose.pose.position.z);
    Vec3d crntPosition = Vec3d(odomCrnt.pose.pose.position.x, odomCrnt.pose.pose.position.y, odomCrnt.pose.pose.position.z);
    prevWorldRt.at<double>(0, 3) = prevPosition[0];
    prevWorldRt.at<double>(1, 3) = prevPosition[1];
    prevWorldRt.at<double>(2, 3) = prevPosition[2];
    crntWorldRt.at<double>(0, 3) = crntPosition[0];
    crntWorldRt.at<double>(1, 3) = crntPosition[1];
    crntWorldRt.at<double>(2, 3) = crntPosition[2];

    //PrintMatrix("prevWorldRt", prevWorldRt);
    //PrintMatrix("crntWorldRt", crntWorldRt);

    Mat relativeRt1 = prevWorldRt.inv() * crntWorldRt;
    Mat relativeRt2 = crntWorldRt.inv() * prevWorldRt;

    /*/ 디버깅
    {
        Vec3d xyz[2], rpy[2];
        MatrixToEulerCCW(relativeRt1, rpy[0][0], rpy[0][1], rpy[0][2], false);
        MatrixToEulerCCW(relativeRt2, rpy[1][0], rpy[1][1], rpy[1][2], false);
        xyz[0][0] = relativeRt1.at<double>(0, 3);
        xyz[0][1] = relativeRt1.at<double>(1, 3);
        xyz[0][2] = relativeRt1.at<double>(2, 3);
        xyz[1][0] = relativeRt2.at<double>(0, 3);
        xyz[1][1] = relativeRt2.at<double>(1, 3);
        xyz[1][2] = relativeRt2.at<double>(2, 3);

        Vec3d deltaRotation = crntRotationEuler - prevRotationEuler;
        Vec3d deltaPosition = crntPosition - prevPosition;

        printf("Direct Odom Relative :\n");
        printf("%7.3lf %7.3lf / %7.3lf\n", deltaPosition[0], deltaPosition[1], deltaRotation[0]);
        printf("Relative #1 :\n");
        printf("%7.3lf %7.3lf / %7.3lf\n", xyz[0][0], xyz[0][1], rpy[0][2]);
        printf("Relative #2 :\n");
        printf("%7.3lf %7.3lf / %7.3lf\n\n\n\n\n\n", xyz[1][0], xyz[1][1], rpy[1][2]);
    }*/

    return relativeRt1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
PrintLaserScanInfo(char *pMsg, sensor_msgs::LaserScan &scan, bool bUseDegree)
{
    if(bUseDegree) {
        printf("%s", pMsg);
        printf("angle_min = %f / angle_max = %f\n", ToDegree(scan.angle_min), ToDegree(scan.angle_max));
        printf("angle_increment = %f, time_increment = %f, scan_time = %f\n", ToDegree(scan.angle_increment), scan.time_increment, scan.scan_time);
        printf("range_min = %f, range_max = %f\n", scan.range_min, scan.range_max);
        printf("Size of ranges = %d, Size of intensities %d\n\n", scan.ranges.size(), scan.intensities.size());
    } else {
        printf("%s", pMsg);
        printf("angle_min = %f / angle_max = %f\n", scan.angle_min, scan.angle_max);
        printf("angle_increment = %f, time_increment = %f, scan_time = %f\n", scan.angle_increment, scan.time_increment, scan.scan_time);
        printf("range_min = %f, range_max = %f\n", scan.range_min, scan.range_max);
        printf("Size of ranges = %d, Size of intensities %d\n\n", scan.ranges.size(), scan.intensities.size());
    }

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
bool
DrawPolarScan(Mat &display, sensor_msgs::LaserScan &scan, Scalar color, bool bClear, int nOffset)
{
    int nSize = (int) scan.ranges.size();

    if(nSize <= 0) {
        return false;
    }

    if(bClear) {
        display = Mat::zeros(nSize, nSize, CV_8UC3);
        line(display, Point(nSize/2-1, 0),Point(nSize/2-1, nSize-1), CV_RGB(255, 255, 0));
        //line(display, Point(nSize/2-1, 0),Point(nSize/2-1, nSize-1), CV_RGB(0, 255, 255));
    }

    for(int i=0; i<nSize; i++) {
        int nIdx = i+nOffset;

        if(nIdx < 0) {                   // 만약 음수인경우
            nIdx = nSize - nIdx;
        } else if(nIdx >= nSize) {       // nSize를 넘어가는 경우
            nIdx = nIdx % nSize;
        } else {
            ;
        }

        if(!isnan(scan.ranges[i]) && scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max) {
            int nPixelDepth = cvRound((scan.ranges[i]/scan.range_max)*(nSize-1));
            display.at<Vec3b>(nSize-1-nPixelDepth, display.cols-1-nIdx) = Vec3b(color[0], color[1], color[2]);
        }
    }

    return true;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
bool
DrawCartesianScan(Mat &display, sensor_msgs::LaserScan &scan, Scalar color, bool bClear, int nOffset)
{
    int nSize = (int) scan.ranges.size();

    if(nSize <= 0) {
        return false;
    }

    if(bClear) {
        display = Mat::zeros(nSize, nSize, CV_8UC3);
        line(display, Point(nSize/2-1, 0),Point(nSize/2-1, nSize-1), CV_RGB(255, 255, 0));
        //line(display, Point(nSize/2-1, 0),Point(nSize/2-1, nSize-1), CV_RGB(0, 255, 255));
    }

    for(int i=0; i<nSize; i++) {
        int nIdx = i+nOffset;

        if(nIdx < 0) {                   // 만약 음수인경우
            nIdx = nSize - nIdx;
        } else if(nIdx >= nSize) {       // nSize를 넘어가는 경우
            nIdx = nIdx % nSize;
        } else {
            ;
        }

        if(!isnan(scan.ranges[i]) && scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max) {
            //int nPixelDepth = cvRound((scan.ranges[i]/scan.range_max)*(nSize-1));
            //display.at<Vec3b>(nSize-1-nPixelDepth, display.cols-1-nIdx) = Vec3b(color[0], color[1], color[2]);


        }
    }

    return true;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
Mat
getWorldRt(nav_msgs::Odometry &odom)
{
    // 변수선언
    Mat worldRt = Mat::eye(4, 4, CV_64F);

    // 회전저장
    Vec3d rotationEuler;
    tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3(rotationQuat).getEulerYPR(rotationEuler[0], rotationEuler[1], rotationEuler[2]);

    Mat R = EulerToMatrixCCW(rotationEuler[2], rotationEuler[1], rotationEuler[0], false);

    for(int r=0; r<3; r++) {
        for(int c=0; c<3; c++) {
            worldRt.at<double>(r, c) = R.at<double>(r, c);
        }
    }

    // 위치저장
    worldRt.at<double>(0, 3) = odom.pose.pose.position.x;
    worldRt.at<double>(1, 3) = odom.pose.pose.position.y;
    worldRt.at<double>(2, 3) = odom.pose.pose.position.z;

    return worldRt;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
Mat
DoPolarRegistration(sensor_msgs::LaserScan &prevPolar, sensor_msgs::LaserScan &crntPolar, float fAngleSearchRange)
{
    /////////////////////////////////////////////////////////////////////////////////////////////
    // 환경변수
    const double dGridMaxDist = 4.5;
    const bool bUseRangeDistInfo = false;
    const bool bShowPolarImage = true;
    const bool bSavePolarImageToFile = false;
    const bool bShowCartesianImage = true;

    Mat retRelativeRt = Mat::eye(4, 4, CV_64F);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // 예외처리!
    const int nSize = (int) crntPolar.ranges.size();

    if(nSize <= 0) {
        return retRelativeRt;
    }

    const int nStep = (int) ((fAngleSearchRange/crntPolar.angle_increment) + 0.5f);


    /////////////////////////////////////////////////////////////////////////////////////////////
    // 레이저 스캔의 평균 거리 구하기
    int nCount = 0;
    double dMean = 0.f;
    double dPrevRangeMean = 0.f, dCrntRangeMean = 0.f;

    if(bUseRangeDistInfo) {
        // n-1 프레임
        for(int i=0; i<(int)prevPolar.ranges.size(); i++) {
            if(!isnan(prevPolar.ranges[i]) && prevPolar.ranges[i] >= prevPolar.range_min && prevPolar.ranges[i] <= prevPolar.range_max) {
                dMean += prevPolar.ranges[i];
                nCount++;
            }
        }

        // 예외처리
        if(!nCount) {
            printf("No range data in n-1_th laser scan\n");
            return retRelativeRt;
        } else {
            dPrevRangeMean = dMean / (double) nCount;
        }

        // n 레이저 스캔의 평균 거리 구하기
        nCount = 0;
        dMean = 0.f;

        for(int i=0; i<(int)crntPolar.ranges.size(); i++) {
            if(!isnan(crntPolar.ranges[i]) && crntPolar.ranges[i] >= crntPolar.range_min && crntPolar.ranges[i] <= crntPolar.range_max) {
                dMean += crntPolar.ranges[i];
                nCount++;
            }
        }

        // 예외처리
        if(!nCount) {
            printf("No range data in n_th laser scan\n");
            return retRelativeRt;
        } else {
            dCrntRangeMean = dMean / (double) nCount;
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////
    // 루프를 돌면서 SAD를 이용한 회전 찾기!!!!
    int nBestStep = -9999999;
    double dBestCorrelation = 9999999.;
    double dBestCorrelationMean = 9999999.;
    Vec3d prevCentroid = Vec3d(0., 0., 0.), crntCentroid = Vec3d(0., 0., 0.);

    vector<Vec3d> laserScanXYPrevious, laserScanXYCurrent, laserScanXYCurrentInitR;
    convertScan2XYZs(prevPolar, laserScanXYPrevious);
    convertScan2XYZs(crntPolar, laserScanXYCurrent);

    for(int i=-nStep; i<=nStep; i++) {
        double dCorrelation = 0.000000001;
        int nCorrelationCount = 0;
        int nPrevCentCount = 0, nCrntCentCount = 0;
        Vec3d tmpPrevCentroid = Vec3d(0., 0., 0.), tmpCrntCentroid = Vec3d(0., 0., 0.);

        for(int nSrcIdx=0; nSrcIdx<nSize; nSrcIdx++) {
            int nDstIdx = nSrcIdx+i;

            if(nDstIdx < 0) {                   // 만약 음수인경우
                nDstIdx = nSize - nDstIdx;
            } else if(nDstIdx >= nSize) {       // nSize를 넘어가는 경우
                nDstIdx = nDstIdx % nSize;
            } else {
                float dPrevDist = 0., dCrntDist = 0.;

                if( !isnan(prevPolar.ranges[nSrcIdx]) ) {
                    dPrevDist = (prevPolar.ranges[nSrcIdx]-dPrevRangeMean);
                    tmpPrevCentroid += laserScanXYPrevious[nSrcIdx];
                    nPrevCentCount++;
                }

                if( !isnan(crntPolar.ranges[nDstIdx]) ) {
                    dCrntDist = (crntPolar.ranges[nDstIdx]-dCrntRangeMean);
                    tmpCrntCentroid += laserScanXYCurrent[nDstIdx];
                    nCrntCentCount++;
                }

                dCorrelation += abs(dPrevDist-dCrntDist);
                nCorrelationCount++;
            }
        }

        if(dCorrelation == 0.f && nCorrelationCount > 0) {
            dBestCorrelation = dCorrelation;
            dBestCorrelationMean = dCorrelation / (double) nCorrelationCount;
            nBestStep = i;

            prevCentroid = tmpPrevCentroid / (double) nPrevCentCount;
            crntCentroid = tmpCrntCentroid / (double) nCrntCentCount;

            break;
        } else if(dCorrelation > 0.f && dCorrelation <= dBestCorrelation && nCorrelationCount > 0) {
            dBestCorrelation = dCorrelation;
            dBestCorrelationMean = dCorrelation / (double) nCorrelationCount;
            nBestStep = i;

            prevCentroid = tmpPrevCentroid / (double) nPrevCentCount;
            crntCentroid = tmpCrntCentroid / (double) nCrntCentCount;
        } else {
            ;
        }
    }

    // 영상 좌표계에서 최소-최대가 실제 3차원 좌표계의 최소-최대와 반대이므로 부호를 반대로 바꿔야 함!
    nBestStep *= -1;
    double dTheta = (nBestStep)*crntPolar.angle_increment;


    /////////////////////////////////////////////////////////////////////////////////////////////
    // 결과 정리
    if(nBestStep == -9999999) {
        printf("Failed!!!\n");
        return retRelativeRt;
    } else {
        printf("BestAngle = %.2f / BestCorrelation = %.2lf\n", ToDegree(nBestStep*crntPolar.angle_increment), dBestCorrelation);
        printf("%d ~ %d / %d\n\n\n", nStep, -nStep, nBestStep);
    }


    /////////////////////////////////////////////////////////////////////////////////////////////
    // 리턴할 Rt 생성
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat::zeros(3, 1, CV_64F);
    Vec3d translation = Vec3d(0, 0, 0);

    double dCosTheta = cos(dTheta);
    double dSinTheta = sin(dTheta);

    R.at<double>(0, 0) = dCosTheta;
    R.at<double>(0, 1) = -dSinTheta;
    R.at<double>(1, 0) = dSinTheta;
    R.at<double>(1, 1) = dCosTheta;

    Mat _prevCentroid = Mat::zeros(3, 1, CV_64F);
    Mat _crntCentroid = Mat::zeros(3, 1, CV_64F);

    _prevCentroid.at<double>(0, 0) = prevCentroid[0];   _prevCentroid.at<double>(1, 0) = prevCentroid[1];   _prevCentroid.at<double>(2, 0) = prevCentroid[2];
    _crntCentroid.at<double>(0, 0) = crntCentroid[0];   _crntCentroid.at<double>(1, 0) = crntCentroid[1];   _crntCentroid.at<double>(2, 0) = crntCentroid[2];

    t = _prevCentroid - R*_crntCentroid;

    translation[0] = t.at<double>(0, 0);
    translation[1] = t.at<double>(1, 0);
    translation[2] = t.at<double>(2, 0);

    for(int r=0; r<3; r++) {
        for(int c=0; c<3; c++) {
            retRelativeRt.at<double>(r, c) = R.at<double>(r, c);
        }

        retRelativeRt.at<double>(r, 3) = t.at<double>(r, 0);
    }


    return retRelativeRt;
}





int
transformRelativeRT(Mat Rt,sensor_msgs::LaserScan &currScan)
{
    int nSize = (int) currScan.ranges.size();
    double x,y;

    Mat ScanPointXY = Mat::eye(4, 1, CV_64F);
    Mat ScanPointXY_Rel = Mat::eye(4, 1, CV_64F);

    for (int i = 0;i < nSize;i++)
    {
        if(isnan(currScan.ranges[i]))  continue;

        double angle = currScan.angle_min + i*currScan.angle_increment;
        ScanPointXY.at<double>(0, 0) = currScan.ranges[i]*cos(angle);
        ScanPointXY.at<double>(1, 0) = currScan.ranges[i]*sin(angle);
        ScanPointXY.at<double>(2, 0) = 0.0;
        ScanPointXY.at<double>(3, 0) = 1.0;

        ScanPointXY_Rel = Rt*ScanPointXY;

        currScan.ranges[i] = sqrt(ScanPointXY_Rel.at<double>(0, 0)*ScanPointXY_Rel.at<double>(0, 0) + ScanPointXY_Rel.at<double>(1, 0)*ScanPointXY_Rel.at<double>(1, 0));
    }
 }





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtle_odom_polar_alignment");
    ros::NodeHandle nh;

    // Create subscriber objects
    ros::Subscriber subOdom = nh.subscribe("/odom", 100, &odomMsgCallback);
    ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);

    // Display buffer
    Mat display;
    initGrid(display, 801);

    // Odometry buffer
    nav_msgs::Odometry odom[2];

    // Scan buffer
    sensor_msgs::LaserScan laserScanPolar[2];

    Mat odomWorldRt[2];

    // 이동 및 회전 정보
    Vec3d xyz, rpy;

    // 이동궤적prevPolar
    vector<Vec3d> trajectory;

    // LRF scan 정보
    vector<Vec3d> laserScanXY[2], laserScanWorldXY[2];
    vector< vector<Vec3d> > laserScanXYMulti;

    // Mat distance for grid
    const double dGridMaxDist = 4.5;

    // 프레임 카운터
    int nFrameCount = 0;

    // 몇 프레임 만에 자동으로 지도 업데이트??
    const int nMapUpdateInterval = 30;

    // 쌍정합을 위한 변수
    int nPrevious = 0, nCurrent = 1;
    bool bFirst = true;

    // 초기화를 위해 더미 프레임을 받아옴!!
    while(ros::ok()) {
        if(laserScanPolar[nCurrent].ranges.size() != 0) {
            break;
        }

        // callback 함수을 call!
        ros::spinOnce();

        // receive the global '/odom' message with the mutex
        mutex[0].lock(); {
            odom[nCurrent] = g_odom;
            odomWorldRt[nCurrent] = getWorldRt(odom[nCurrent]);
        } mutex[0].unlock();

        // receive the global '/scan' message with the mutex
        mutex[1].lock(); {
            laserScanPolar[nCurrent] = g_scan;
        } mutex[1].unlock();

        nPrevious ^= 1;
        nCurrent ^= 1;
    }

    // main loop
    while(ros::ok()) {
        // callback 함수을 call!
        ros::spinOnce();

        // receive the global '/odom' message with the mutex
        mutex[0].lock(); {
           odom[nCurrent] = g_odom;
           odomWorldRt[nCurrent] = getWorldRt(odom[nCurrent]);
        } mutex[0].unlock();

        // receive the global '/scan' message with the mutex
        mutex[1].lock(); {
            laserScanPolar[nCurrent] = g_scan;
        } mutex[1].unlock();

        if(bFirst) {            // 만약 첫 프레임이라면 그냥 지나치고...
            bFirst = false;
        } else {                // 두번째 프레임부터 정합 시작!!
            // odom으로 n-1 프레임과 n 프레임 사이의 상대변환 획득
            Mat relativeOdomRt = odomWorldRt[nPrevious].inv() * odomWorldRt[nCurrent];

            transformRelativeRT(relativeOdomRt,laserScanPolar[nCurrent]);

            // 현재 프레임 사이의 correlation이 가장 놓은 theta 찾기
            Mat newRelativeRt = DoPolarRegistration(laserScanPolar[nPrevious], laserScanPolar[nCurrent], ToRadian(30));

            // 보정된 새로운 월드 만들기
            odomWorldRt[nCurrent] = odomWorldRt[nPrevious]*newRelativeRt*relativeOdomRt;

            MatrixToEulerCCW(odomWorldRt[nCurrent], rpy[0], rpy[1], rpy[2], false);
            xyz[0] = odomWorldRt[nCurrent].at<double>(0, 3);
            xyz[1] = odomWorldRt[nCurrent].at<double>(1, 3);
            xyz[2] = odomWorldRt[nCurrent].at<double>(2, 3);

            // 현재의 위치를 저장
            saveCurrentPosition(xyz, trajectory, 0.02);

            // scan으로부터 Cartesian X-Y scan 획득
            convertScan2XYZs(laserScanPolar[nCurrent], laserScanXY[nCurrent]);

            // laserScan을 월드좌표계로 변환
            transform(laserScanXY[nCurrent], laserScanWorldXY[nCurrent], xyz[0], xyz[1], rpy[2]);

            // 지도 자동 업데이트!
            laserScanXYMulti.push_back(laserScanWorldXY[nCurrent]);

            // 현재 상황을 draw할 display 이미지를 생성
            initGrid(display, 801);
            drawTrajectory(display, trajectory, dGridMaxDist);
            drawCurrentPositionWithRotation(display, xyz, rpy, dGridMaxDist);
            drawLRFScanMulti(display, laserScanXYMulti, dGridMaxDist);
            drawLRFScan(display, laserScanWorldXY[nCurrent], dGridMaxDist, CV_RGB(0, 255, 255));

            // 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
            transpose(display, display);        // X-Y축 교환
            flip(display, display, 0);          // 수평방향 반전
            flip(display, display, 1);          // 수직방향 반전

            // 영상 출력!
            imshow("KNU ROS Lecture >> turtle_position_lrf_view", display);

            // 사용자의 키보드 입력을 받음!
            int nKey = waitKey(100) % 255;

            if(nKey == 27) {
                // 종료
                break;
            } else if(nKey == 'c' || nKey == 'C') {
                initGrid(display, 801);
                laserScanXYMulti.clear();
                trajectory.clear();
            }
        }

        // 프레임 토글링 및 카운터 증가
        nPrevious ^= 1;
        nCurrent ^= 1;
        nFrameCount++;
    }

    return 0;
}
