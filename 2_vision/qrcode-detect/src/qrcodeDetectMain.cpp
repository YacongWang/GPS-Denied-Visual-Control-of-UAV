#include "qrcodeDetectMain.h"
using namespace qrcodeDdetect;
using namespace std;
using namespace cv;
#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

QrcodeDdetect::QrcodeDdetect(ros::NodeHandle nodehandle, image_transport::ImageTransport it)
    : nodehandle_(nodehandle),
      it_(it),
      imgMsg_(new sensor_msgs::Image),
      poseArrayMsg_(new geometry_msgs::PoseArray)
    /*,calibFile_(calibFile)*/
{
    readParameters();
    pubImg_ = it.advertise("/qrDetectImg", 1);
    pubMarkerPoses_ = nodehandle_.advertise<geometry_msgs::PoseArray> ("/qrcodePose", 1);
    toDegree = 180/3.14159;
}

void QrcodeDdetect::readParameters()
{
    //readCalibPara(calibFile_.c_str());
    nodehandle_.param("strSub_img",    strSub_img_,     string("/camera/image_color"));
}
void QrcodeDdetect::cfgCallback(const Parameter cfg)
{
    cfg_ = cfg;
}
void QrcodeDdetect::img_Callback(const sensor_msgs::ImageConstPtr& img)
{
    /************************** get image data **************************/
    grabImg(img);
//    Size size(340,240);//the dst image size,e.g.100x100
//    resize(img_,img_,size);//resize image
    markCapture_.processFrame(img_);
    pc2Time_ = img->header.stamp;
    cvtColor(img_, cImg_, CV_GRAY2RGB);
    poseArrayMsg_->poses.clear();
    if(markCapture_.m_markers.size())
    {
        // show marker in image

        for(int i=0; i<markCapture_.m_markers.size(); i++)
        {
            int sizeNum = markCapture_.m_markers[i].m_points.size();
            for (int j=0; j<sizeNum; j++)
            {
                line(cImg_, markCapture_.m_markers[i].m_points[j], markCapture_.m_markers[i].m_points[(j+1)%sizeNum], Scalar(0,0,255), 10, 8);
            }
            circle(cImg_, markCapture_.m_markers[i].m_points[0], 3, Scalar(0,255,255), 10, 8);//clockwise code the points
            addpose(markCapture_.m_markers[i].m_rotation, markCapture_.m_markers[i].m_translation);

            char bufTran[50],bufRot[50];
//            sprintf(bufTran,"x: %5.4f, y: %5.4f, z: %5.4f\nroll: %5.4f, pitch: %5.4f, yaw: %5.4f",
//                    markerPose_.position.x,markerPose_.position.y,markerPose_.position.z,
//                    markerPose_.orientation.x,markerPose_.orientation.y,markerPose_.orientation.z);
            sprintf(bufTran,"x: %5.4f, y: %5.4f, z: %5.4f",
                    markerPose_.position.x,markerPose_.position.y,markerPose_.position.z);
            sprintf(bufRot,"roll: %5.4f, pitch: %5.4f, yaw: %5.4f",
                    markerPose_.orientation.x,markerPose_.orientation.y,markerPose_.orientation.z);
            putText(cImg_, bufTran, Point(20,img_.rows - 80), cv::FONT_HERSHEY_DUPLEX, 1.5, Scalar(255,0,0), 2);
//            Point texCentre;
//            texCentre.x =  markCapture_.m_markers[i].m_points[0].x;
//            texCentre.y =  markCapture_.m_markers[i].m_points[0].y + 20;
//            putText(cImg_, bufRot, texCentre, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0), 2);
            putText(cImg_, bufRot, Point(20,img_.rows - 20), cv::FONT_HERSHEY_DUPLEX, 1.5, Scalar(255,0,0), 2);

        }

        //imshow("markerDetector", cImg);
        //cv::waitKey(5);

    }
    else
    {

    }
     publishMsg();
    //imageCloudPoints_<< imgPoint_.x <<" "<<imgPoint_.y<<endl;

}

void QrcodeDdetect::grabImg(const sensor_msgs::ImageConstPtr& img)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }   
        //Here we get the cv::Mat message ,that is cv_ptr->image
      cv_ptr->image.copyTo(img_);
}
void QrcodeDdetect::addpose(Vec3f eulerAngle , Vec3f tran)
{

      markerPose_.position.x = tran[1];
      markerPose_.position.y = -tran[0];
      markerPose_.position.z = -tran[2];

      markerPose_.orientation.x = eulerAngle[1] * toDegree;
      markerPose_.orientation.y = -eulerAngle[0] * toDegree;
      markerPose_.orientation.z = -eulerAngle[2] * toDegree;
      //markerPose.orientation.w = quat.w();
      //ROS_INFO_STREAM("Added block: \n" << block_pose );
      poseArrayMsg_->poses.push_back(markerPose_);
}

void QrcodeDdetect::publishMsg()
{
    //publish pose
    poseArrayMsg_->header.frame_id = "camera";
    poseArrayMsg_->header.stamp = pc2Time_;
    pubMarkerPoses_.publish(poseArrayMsg_);

    //publish image
    imgMsg_ = cv_bridge::CvImage(poseArrayMsg_->header, "bgr8", cImg_).toImageMsg();
    pubImg_.publish(imgMsg_);
    cv::waitKey(1);
}
//int QrcodeDdetect::readCalibPara(string filename)
//{
//    cv::FileStorage fs(filename,cv::FileStorage::READ);
//    if(!fs.isOpened())
//    {
//        std::cout<<"Invalid calibration filename."<<std::endl;
//        return 0;
//    }
//    fs[CAMERAMAT]>>cameraMat_;
//    fs[DISTCOEFF]>>distCoeff_;
//    fs[CAMERAEXTRINSICMAT]>>cameraExtrinsicMat_;
//    fs[IMAGESIZE]>>imageSize_;
////    R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3)).t();
////    T_ = -R_*cameraExtrinsicMat_(cv::Rect(3,0,1,3));
//    R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3));
//    T_ = cameraExtrinsicMat_(cv::Rect(3,0,1,3));
//    cout<<"cameraMat:"<<cameraMat_<<endl;
//    cout<<"distCoeff:"<<distCoeff_<<endl;
//    cout<<"cameraExtrinsicMat:"<<cameraExtrinsicMat_<<endl;
//    cout<<"imageSize:"<<imageSize_<<endl;
//}







