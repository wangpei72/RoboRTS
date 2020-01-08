//
// Created by wang_shuai on 19-12-27.
//
#include <opencv2/opencv.hpp>
#include "mvs_driver.h"



namespace roborts_camera
{
 MVS_Driver::MVS_Driver(roborts_camera::CameraInfo cameraInfo_):CameraBase(cameraInfo_)
 {
     int n_Ret = 1;
     MV_CC_DEVICE_INFO_LIST stDeviceList;
     n_Ret = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);

     MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];

     n_Ret = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);

     n_Ret = MV_CC_OpenDevice(handle);

     SetEnumValue("TriggerMode",0);

     n_Ret = MV_CC_StartGrabbing(handle);

     camera_initialized_ = n_Ret;

 }

 void MVS_Driver::StartReadCamera(cv::Mat &img)
 {
     int n_Ret = 1;

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};

    free(img.data);

    n_Ret = MV_CC_GetImageForBGR(handle,img.data,maxDataSize,&stImageInfo,100);

    img.rows = stImageInfo.nHeight;

    img.cols = stImageInfo.nWidth;

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

