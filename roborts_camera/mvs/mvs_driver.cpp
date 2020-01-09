//
// Created by wang_shuai on 19-12-27.
//
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "mvs_driver.h"



namespace roborts_camera
{
 MVS_Driver::MVS_Driver(roborts_camera::CameraInfo cameraInfo_):CameraBase(cameraInfo_)
 {
     int n_Ret = 1;
     MV_CC_DEVICE_INFO_LIST stDeviceList;
     n_Ret = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);

     MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];

     //ROS_INFO("%d***********",stDeviceList.nDeviceNum);

     n_Ret = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
     //ROS_INFO("%d*********",n_Ret);
     n_Ret = MV_CC_OpenDevice(handle);
     //ROS_INFO("%d-------%d",n_Ret,MV_OK);

     SetEnumValue("TriggerMode",0);

     n_Ret = MV_CC_StartGrabbing(handle);

     ROS_INFO("MVS Start GRABBING state :%d", n_Ret == MV_OK);

     camera_initialized_ = 1;

 }

 void MVS_Driver::StartReadCamera(cv::Mat &img , cv::Mat depth)
 {
     int n_Ret = 1;

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};

    img.create(camera_info_.resolution_height,
                camera_info_.resolution_width,
                CV_8UC3);

    //free(img.data);

    //img.data = 0;

    uchar *ptr = (unsigned char *)malloc(sizeof(unsigned char) * maxDataSize);



    n_Ret = MV_CC_GetImageForBGR(handle,ptr,maxDataSize,&stImageInfo,1000);


    if (n_Ret)
    {
        ROS_INFO("MVS failed");
        exit;
    }

    mempcpy(img.data,ptr,stImageInfo.nHeight*stImageInfo.nWidth*3);

    img.rows = stImageInfo.nHeight;

    img.cols = stImageInfo.nWidth;

    free(ptr);
    

    /*
    ROS_INFO("gain : %f",stImageInfo.fGain);
    ROS_INFO("exposure time : %f",stImageInfo.fExposureTime);
    ROS_INFO("X : %d",stImageInfo.nOffsetX);
    ROS_INFO("Y : %d",stImageInfo.nOffsetY);
    ROS_INFO("bright : %d",stImageInfo.nAverageBrightness);
     */
 };

 void MVS_Driver::StopReadCamera()
 {
     int n_Ret = 1;

     n_Ret = MV_CC_StopGrabbing(handle);

     n_Ret = MV_CC_CloseDevice(handle);

     n_Ret = MV_CC_DestroyHandle(handle);
 }

 void MVS_Driver::SetEnumValue(std::string name, int value)
 {
     MV_CC_SetEnumValue(handle,name.c_str(), 0);
 }

 void MVS_Driver::WriteParam()
 {

 }

 void MVS_Driver::LoadParam()
 {

 }

 MVS_Driver::~MVS_Driver()
 {

 }

}

