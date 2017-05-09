#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "uav/CtrlParamConfig.h"
#include <dynamic_reconfigure/server.h>
#include <uav/CtrlParamConfig.h>
#include <image_transport/image_transport.h>
//!opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace DJI::onboardSDK;
using namespace cv;

cv::Mat gImg;                   //grab image
int w = 640, h = 600;
cv::Mat gImgShow = Mat::zeros( h, w, CV_8UC3);
int main_operate_code = 0;
int buttonColor = 0;
std::string winName("UAV");
geometry_msgs::Pose visionPos_;

struct Parameter
{
    double scaleX;
    double scaleY;
    double scaleZ;
    double scaleYaw;
    double disRange;
};

Parameter param;
void Callback(uav::CtrlParamConfig &config, uint32_t level)
{
    param.scaleX           = config.scaleX;
    param.scaleY           = config.scaleY;
    param.scaleZ           = config.scaleZ;
    param.scaleYaw         = config.scaleYaw;
    param.disRange         = config.disRange;
}

void posRange(double& pos)
{
//    double pos = Pos;
    if(pos > param.disRange)
        pos = param.disRange;
    else if(pos < - param.disRange)
        pos = - param.disRange;


}


void posToAng(double& pos, double& Angle)
{
//    double pos = Pos;
//    Angle = 0;
//    if(pos > param.disRange)
//        pos = param.disRange;
//    else if(pos < - param.disRange)
//        pos = - param.disRange;
//    else
//    {
    posRange(pos);
    if(abs(pos)!= param.disRange)
        Angle = 30*pos/param.disRange;
    else Angle = 0;
//    }


}

void vPosCallback(const geometry_msgs::PoseArray& msg)
{
    if(!msg.poses.empty())
    {
        visionPos_.position.x  = msg.poses[0].position.x;
        visionPos_.position.y  = -msg.poses[0].position.y;
        visionPos_.position.z  = msg.poses[0].position.z;
        posToAng(visionPos_.position.x, visionPos_.orientation.y);
//        cout<<"pitch: "<<visionPos_.orientation.y<<endl<<endl;
        posToAng(visionPos_.position.y, visionPos_.orientation.x);
//        cout<<"roll: "<<visionPos_.orientation.x<<endl<<endl;
        posRange(visionPos_.position.z);
//        cout<<"z: "<<visionPos_.position.z<<endl<<endl;
        visionPos_.orientation.z = msg.poses[0].orientation.z;
    //    char message[40];
    //    sprintf(message, "%d %6.4f %6.4f %6.4f",
    //          msg.poses.size(),
    //          msg.poses[0].position.x,
    //          msg.poses[0].position.y,
    //          msg.poses[0].position.z);
    //    cout<<message<<endl;

    //    node_handle.loginfo(message);

    }
}

void cavansInit()
{
     rectangle( gImgShow,
         Point( 0, 520 ),
         Point( w, h),
         Scalar( 224 ,238 ,224 ),
         -1,
         8 );
    line( gImgShow, Point( 0, 14*h/15 ), Point( w, 14*h/15 ) , Scalar( 0, 0, 0 ),2,8);
    line( gImgShow, Point( w/4, 13*h/15 ), Point( w/4, h ) , Scalar( 0, 0, 0 ),2,8);
    line( gImgShow, Point( w/2, 13*h/15 ), Point( w/2, h ) , Scalar( 0, 0, 0 ),2,8);
    line( gImgShow, Point( 3*w/4, 13*h/15 ), Point( 3*w/4, h ) , Scalar( 0, 0, 0 ),2,8);
    putText(gImgShow, "request control", Point(0+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
    putText(gImgShow, "take off", Point(w/4+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
    putText(gImgShow, "visual tracking", Point(w/2+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
    putText(gImgShow, "release control", Point(3*w/4+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
}


void switchColor(int val)
{
    switch(val)
    {
//    case 0:
//        cavansInit();
//        break;
    case 1:
        /* request control ability*/
        rectangle( gImgShow, Point( 5, 13*h/15+5 ), Point(w/4-5, 14*h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "request control", Point(0+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    case 2:
        /* take off */
        rectangle( gImgShow, Point( w/4+5, 13*h/15+5 ), Point(w/2-5, 14*h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "take off", Point(w/4+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    case 3:
        /* vision tracking mission */
        rectangle( gImgShow, Point( w/2+5, 13*h/15+5 ), Point(3*w/4-5, 14*h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "visual tracking", Point(w/2+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    case 4:
        /* release control ability*/
        rectangle( gImgShow, Point( 3*w/4+5, 13*h/15+5 ), Point(w-5, 14*h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "release control", Point(3*w/4+10,13*h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    default:
        break;
    }

}

void OnMouseAction(int event,int x,int y,int flags,void *ustc)
{

    if(event==CV_EVENT_LBUTTONDOWN) //left button down
    {
        if(y>13*h/15 && y <14*h/15)
        {
            switch(x/150)
            {
            case 0:
                /* request control ability*/
                main_operate_code = 1;
                buttonColor = 1;
                break;
            case 1:
                /* take off */
                main_operate_code = 2;
                buttonColor = 2;
                break;
            case 2:
                /* vision tracking mission */
                main_operate_code = 3;
                buttonColor = 3;
                break;
            case 3:
                /* release control ability*/
                main_operate_code = 4;
                buttonColor = 4;
                break;
            default:
                break;

            }

        }

    }
    if(event==CV_EVENT_LBUTTONUP)   //left button up
    {
        switchColor(buttonColor);
//        imshow(winName, gImgShow);
//        cv::waitKey(1);
    }
}


void grabImg(const sensor_msgs::ImageConstPtr& img)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img, "bgr8");

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
        //Here we get the cv::Mat message ,that is cv_ptr->image
      cv_ptr->image.copyTo(gImg);
}

void img_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    /************************** get image data **************************/
    grabImg(msg);
    Size size(640,512);//the dst image size,e.g.100x100//1280x1024
    resize(gImg,gImg,size);//resize image
    gImg.copyTo(gImgShow(Rect(0,0,640,512)));
//    imshow(winName, gImgShow);
//    cv::waitKey(5);

}

void imShowSlot()
{
   // switchColor(main_operate_code);
    imshow(winName, gImgShow);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    namedWindow(winName, CV_WINDOW_AUTOSIZE);
    ros::init(argc, argv, "uav_node");
    if(argc!=1)
    {
        cerr<<endl<<"Usage: rosrun uav uav_node";
        ros::shutdown();
        return 1;
    }
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);
    ros::Subscriber sub = nh.subscribe("/qrcodePose", 1, vPosCallback);
    image_transport::ImageTransport iT(nh);
    image_transport::Subscriber subImg = iT.subscribe("/qrDetectImg", 1, img_Callback);
    
    ros::Publisher pubCtrlParam = nh.advertise<geometry_msgs::Pose> ("/ctrlPose", 1);

    /*************************** dynamic_reconfigure ***********************************/
    dynamic_reconfigure::Server<uav::CtrlParamConfig> server;
    dynamic_reconfigure::Server<uav::CtrlParamConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);
    cavansInit();
    setMouseCallback(winName,OnMouseAction);
    //ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        imShowSlot();
        switch(main_operate_code)
        {
            case 1:
                /* request control ability*/
                cout<<"request control ability"<<endl<<endl;
                drone->request_sdk_permission_control();

                break;
            case 2:
                /* take off */
                drone->takeoff();
                cout<<"take off "<<endl<<endl;
                break;
            case 3:
//                cout<<"visual tracking"<<endl<<endl;
                drone->attitude_control(0x02, param.scaleX*visionPos_.orientation.x,
                                               param.scaleY*visionPos_.orientation.y,
                                               param.scaleZ*(0.8+visionPos_.position.z ),
                                               /*0*/param.scaleYaw*visionPos_.orientation.z);
                usleep(20000);//50Hz
                break;
            case 4:
                /* release control ability*/
                cout<<"release control ability"<<endl<<endl;
                drone->release_sdk_permission_control();
                break;
            default:
                break;
        }
        if(main_operate_code !=3 )
        {
            main_operate_code = 0;
        }
        //loop_rate.sleep();

    }
    return 0;
}
