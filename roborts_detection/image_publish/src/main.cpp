//
// Created by wang_shuai on 19-12-22.
//

#include <iostream>
#include  "MvCameraControl.h"
#include <pthread.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#define MAX_IMAGE_DATA_SIZE 1024*1024*40

int g_bExit = 0;






int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_publish");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub = it.advertise("/image/bgr_image",1);

    int nRet = MV_OK;

    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    //memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);

    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];

    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);

    nRet = MV_CC_OpenDevice(handle);

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    //std::cout<<nRet<<std::endl;
    nRet = MV_CC_StartGrabbing(handle);


    while(nh.ok())
    {   cv::Mat src(1080,1440,CV_8UC3);

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};

        src.data = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);


        nRet  = MV_CC_GetImageForBGR(handle,src.data,MAX_IMAGE_DATA_SIZE,&stImageInfo,100);

        sensor_msgs::ImageConstPtr msg =  cv_bridge::CvImage(std_msgs::Header(),"bgr8",src).toImageMsg();

        pub.publish(msg);

        //std::cout<<nRet<<std::endl;

        printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
               stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);

        ros::spinOnce();
/**
        cv::Mat to_display;
        cv::resize(src,to_display,cv::Size(3027,2048));
        //to_display = src(cv::Rect(0,0,720,480)).clone();
        cv::imshow("win",to_display);
        cv::waitKey(10);
**/
        free(src.data);
    }


    if (nRet != 0)
    {
        printf("thread create failed.ret = %d\n",nRet);
        //return -1;
    }





    nRet = MV_CC_StopGrabbing(handle);

    nRet = MV_CC_CloseDevice(handle);

    nRet = MV_CC_DestroyHandle(handle);

    //std::cout<<stDeviceList.nDeviceNum<<std::endl;
}
