#ifndef QRCODEDETECTMAIN_H
#define QRCODEDETECTMAIN_H

#include <ros/ros.h>
//!opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
//!MarkerDetector
#include "MarkerDetector.h"
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <string>

using namespace std;
using namespace cv;

/*!
 * camera coordinate:x~left,y~down,z~front
 */
struct Parameter
{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};
namespace qrcodeDdetect {

class QrcodeDdetect
{
public:
    /*!
     * Constructor.
     */
    QrcodeDdetect (ros::NodeHandle nodehandle, image_transport::ImageTransport it);
    /*!
     * Deconstructor.
     */
    virtual ~QrcodeDdetect () {}
    /*!
     * imagetopic callback function using message filter
     */
    void img_Callback(const sensor_msgs::ImageConstPtr& img);
    /*!
     * dynamic param callback function
     */
    void cfgCallback(const Parameter cfg);
    /*!
     * grab cv::mat from image topic
     */
    void grabImg(const sensor_msgs::ImageConstPtr& img);
    /*!
     * get qrcode center coordinate from image
     */
    void getImgPoint();
    /*!
     * get the calibration param from calib.yml file
     */
    int readCalibPara(string filename);
    /*!
     * publish point cloud message
     */
    void publishMsg();
    //! ROS Subscribe image message
    string strSub_img_;

    void addpose(Vec3f eulerAngle , Vec3f Tran);




private:

    /*!
     * Read the parameters.
     */
    void readParameters();
    //! dynamic config parameters
    Parameter cfg_;
    //! MarkerDetect
    MarkerDetector markCapture_;
    //! ROS nodehandle
    ros::NodeHandle nodehandle_;
    //! ROS Publisher
    ros::Publisher pubMarkerPoses_;
    image_transport::Publisher pubImg_;
    ros::Time pc2Time_;             //pulish message topic time
    sensor_msgs::ImagePtr imgMsg_;
    geometry_msgs::PoseArrayPtr poseArrayMsg_;
    //!image message
    cv::Mat img_;                   //grab image

    //!image calibration param
    cv::Mat cameraMat_;             //camera internal parameter
    cv::Mat distCoeff_;             //camera distortion parameter
    cv::Size imageSize_;            //image size
    cv::Mat cImg_;
    //!read and write param file
    string calibFile_;              //cameta laser calibration file
    image_transport::ImageTransport it_;
    double toDegree;
    geometry_msgs::Pose markerPose_;
//    ofstream imageCloudPoints_;     //QR code board center points in image and laser coordinate
};

}
#endif // 
