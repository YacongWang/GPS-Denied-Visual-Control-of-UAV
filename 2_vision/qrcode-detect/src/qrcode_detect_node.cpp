#include <ros/ros.h>
#include "qrcodeDetectMain.h"
#include <dynamic_reconfigure/server.h>
#include <qrcode_detect/QrcodeDetectConfig.h>

using namespace sensor_msgs;
using namespace std;
Parameter param;

void Callback(qrcode_detect::QrcodeDetectConfig &config, uint32_t level)
{
    param.thresh           = config.thresh;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "qrcode_detect");
    if(argc!=1)
    {
        cerr<<endl<<"Usage: rosrun qrcode_detect qrcode_detect_node";
        ros::shutdown();
        return 1;
    }
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    qrcodeDetect::QrcodeDetect QrcodeDetect(nh,it);
   // ros::spin();
    /*************************** subscribe image ***********************************/
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber rawImage2Cv_sub;
    //rawImage2Cv_sub = it.subscribe("/pg_15508342/image_raw", 1, rawImage2Cv_cbk);
    //ros::Subscriber sub = nh	.subscribe("/pg_15508342/image_rect", 1, &QrcodeDetect::QrcodeDetect::img_Callback, &QrcodeDetect);

    image_transport::Subscriber sub = it.subscribe("/pg_15508342/image_rect", 1, &qrcodeDetect::QrcodeDetect::img_Callback, &QrcodeDetect);

    /*************************** dynamic_reconfigure ***********************************/
    dynamic_reconfigure::Server<qrcode_detect::QrcodeDetectConfig> server;
    dynamic_reconfigure::Server<qrcode_detect::QrcodeDetectConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);
    //ros::Rate loop(5);//figure out this
    while(ros::ok())
    {
        ros::spinOnce();
        QrcodeDetect.cfgCallback(param);
    }

    return 0;
}

