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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace DJI::onboardSDK;
using namespace cv;

struct Parameter
{
    double scaleX;
    double scaleY;
    double scaleZ;
    double scaleYaw;
    double disRange;
    int w;
    int h;
};

struct UAV_status
{
    int  ui_order;      //1：request control 2：request control 3：request control 4：release control
    int  btn_color;     //Change button color after cicked
    int  target_lost_cnt;//Count how many times we have lost the target, if more than 5,then let the uav hold
    bool target_lost;   // 0: found 1:lost

};

Parameter param;
UAV_status uavStatus = {0,0,0,0};
cv::Mat gImg;
cv::Mat gImgShow = Mat::zeros( 600, 640, CV_8UC3);
std::string winName("UAV");
geometry_msgs::Pose visionPos_;


void Callback(uav::CtrlParamConfig &config, uint32_t level)
{
    param.scaleX           = config.scaleX;
    param.scaleY           = config.scaleY;
    param.scaleZ           = config.scaleZ;
    param.scaleYaw         = config.scaleYaw;
    param.disRange         = config.disRange;
    param.w                = config.w;
    param.h                = config.h;
}

void posRange(double& pos)
{
    if(pos > param.disRange)
        pos = param.disRange;
    else if(pos < - param.disRange)
        pos = - param.disRange;
}


void posToAng(double& pos, double& Angle)
{
    posRange(pos);
//    if(abs(pos)!= param.disRange)
        Angle = 30*pos/param.disRange;
//    else Angle = 0;
}

void vPosCallback(const geometry_msgs::PoseArray& msg)
{
    if(!msg.poses.empty())
    {
        if (uavStatus.target_lost_cnt >= 5 && uavStatus.target_lost == 0)
        {
            uavStatus.target_lost = 1;
            uavStatus.target_lost_cnt = 0;
            return;
        }
        if(msg.poses[0].position.x == -100 && msg.poses[0].position.y == -100 && msg.poses[0].position.z == -100)
        {
            uavStatus.target_lost_cnt += 1;
            return;
        }

        uavStatus.target_lost == 0;
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
    }
}

void cavansInit()
{
     rectangle( gImgShow,
         Point( 0, 520 ),
         Point( param.w, param.h),
         Scalar( 224 ,238 ,224 ),
         -1,
         8 );
    line( gImgShow, Point( 0, 14*param.h/15 ), Point( param.w, 14*param.h/15 ) , Scalar( 0, 0, 0 ),2,8);
    line( gImgShow, Point( param.w/4, 13*param.h/15 ), Point( param.w/4, param.h ) , Scalar( 0, 0, 0 ),2,8);
    line( gImgShow, Point( param.w/2, 13*param.h/15 ), Point( param.w/2, param.h ) , Scalar( 0, 0, 0 ),2,8);
    line( gImgShow, Point( 3*param.w/4, 13*param.h/15 ), Point( 3*param.w/4, param.h ) , Scalar( 0, 0, 0 ),2,8);
    putText(gImgShow, "request control", Point(0+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
    putText(gImgShow, "take off", Point(param.w/4+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
    putText(gImgShow, "visual tracking", Point(param.w/2+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
    putText(gImgShow, "release control", Point(3*param.w/4+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
}


void switchColor(int val)
{
    switch(val)
    {
    case 1:
        /* request control ability*/
        rectangle( gImgShow, Point( 5, 13*param.h/15+5 ), Point(param.w/4-5, 14*param.h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "request control", Point(0+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    case 2:
        /* take off */
        rectangle( gImgShow, Point( param.w/4+5, 13*param.h/15+5 ), Point(param.w/2-5, 14*param.h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "take off", Point(param.w/4+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    case 3:
        /* vision tracking mission */
        rectangle( gImgShow, Point( param.w/2+5, 13*param.h/15+5 ), Point(3*param.w/4-5, 14*param.h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "visual tracking", Point(param.w/2+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    case 4:
        /* release control ability*/
        rectangle( gImgShow, Point( 3*param.w/4+5, 13*param.h/15+5 ), Point(param.w-5, 14*param.h/15-5), Scalar( 0, 255, 255 ), -1, 8 );
        putText(gImgShow, "release control", Point(3*param.w/4+10,13*param.h/15+20), cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar(0,0,0), 1);
        break;
    default:
        break;
    }

}

void OnMouseAction(int event,int x,int y,int flags,void *ustc)
{

    if(event==CV_EVENT_LBUTTONDOWN) //left button down
    {
        if(y>13*param.h/15 && y <14*param.h/15)
        {
            switch(x/(param.h/4))
            {
            case 0:
                /* request control ability*/
                uavStatus.ui_order = 1;
                uavStatus.btn_color = 1;
                break;
            case 1:
                /* take off */
                uavStatus.ui_order = 2;
                uavStatus.btn_color = 2;
                break;
            case 2:
                /* vision tracking mission */
                uavStatus.ui_order = 3;
                uavStatus.btn_color = 3;
                break;
            case 3:
                /* release control ability*/
                uavStatus.ui_order = 4;
                uavStatus.btn_color = 4;
                break;
            default:
                break;

            }

        }

    }
    if(event==CV_EVENT_LBUTTONUP)   //left button up
    {
        switchColor(uavStatus.btn_color);
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
    resize(gImg,gImg,Size(640,512));//resize image
    gImg.copyTo(gImgShow(Rect(0,0,640,512)));
}

void imShowSlot()
{
    imshow(winName, gImgShow);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    namedWindow(winName, CV_WINDOW_AUTOSIZE);    //image show window

    /*************************** ros init ***********************************/
    ros::init(argc, argv, "uav_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/qrcodePose", 1, vPosCallback);     //subscribe position msg
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg = it.subscribe("/qrDetectImg", 1, img_Callback);     //subscribe image msg
    //ros::Rate loop_rate(50);

    /*************************** m100 init ***********************************/
    DJIDrone* drone = new DJIDrone(nh);

    /*************************** dynamic_reconfigure ***********************************/
    dynamic_reconfigure::Server<uav::CtrlParamConfig> server;
    dynamic_reconfigure::Server<uav::CtrlParamConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);

    /*************************** OpenCV UI init ***********************************/
    cavansInit();
    setMouseCallback(winName,OnMouseAction);

    /*************************** main loop: check ui order ***********************************/
    while(ros::ok())
    {
        ros::spinOnce();
        imShowSlot();

        switch(uavStatus.ui_order)
        {
            case 1:
                /* request control ability*/
                cout<<"request control ability"<<endl<<endl;
                drone->request_sdk_permission_control();
                break;
            case 2:
                /* take off */
                cout<<"take off "<<endl<<endl;
                drone->takeoff();
                break;
            case 3:
                /*cout<<"visual tracking"<<endl<<endl;*/
                if(uavStatus.target_lost == 0)
                {
                    drone->attitude_control(0x02, param.scaleX*visionPos_.orientation.x,
                                                   param.scaleY*visionPos_.orientation.y,
                                                   param.scaleZ*(0.8+visionPos_.position.z ),
                                                   /*0*/param.scaleYaw*visionPos_.orientation.z);
                    usleep(20000);  //50Hz
                }
                else if (uavStatus.target_lost == 1)
                {
                    drone->attitude_control(0x02, 0, 0, 0, 0);
                    usleep(20000);  //50Hz
                }

                break;
            case 4:
                /* release control ability*/
                cout<<"release control ability"<<endl<<endl;
                drone->release_sdk_permission_control();
                break;
            default:
                break;
        }
        if(uavStatus.ui_order !=3 )
        {
            uavStatus.ui_order = 0;
        }
        //loop_rate.sleep();
    }
    return 0;
}
