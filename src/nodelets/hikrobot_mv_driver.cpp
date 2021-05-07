/*
 * Copyright © 2021 , Universite Bretagne Sud, Lab-STICC, Ronan Douguet
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pluginlib/class_list_macros.h>

#include "hikrobot_mv_ros/hikrobot_mv_driver.h"

PLUGINLIB_EXPORT_CLASS(hikrobot_mv_ros::HikrobotMVDriver, nodelet::Nodelet)

using namespace cv;
using namespace hikrobot_mv_ros;

/* public methods -----------------------------------------------------------*/
//=============================================================================
HikrobotMVDriver::HikrobotMVDriver()
{
}

HikrobotMVDriver::~HikrobotMVDriver()
{
  closeCamera();
}


/* private methods ----------------------------------------------------------*/
//=============================================================================
/**
  @fn void HikrobotMVDriver::onInit()
    Module to init the hikrobot mv camera
*/
void HikrobotMVDriver::onInit()
{
  int mv_res = MV_OK;  
  int index = 0;  
  bool exit = false;  
  
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  
  nh = getNodeHandle();
  
  // get the parameters (load via the camera.yaml file) -------------
  nh.getParam("/camera_mv/width", width);
  nh.getParam("/camera_mv/height", height);  
  nh.getParam("/camera_mv/Offset_x", offset_x);
  nh.getParam("/camera_mv/Offset_y", offset_y);  
  nh.getParam("/camera_mv/FrameRateEnable", frame_rate_enable);
  nh.getParam("/camera_mv/FrameRate", frame_rate);  
  nh.getParam("/camera_mv/BurstFrameCount", burst_frame_count);     
  nh.getParam("/camera_mv/ExposureAuto", exposure_auto);       
  nh.getParam("/camera_mv/ExposureTime", exposure_time);    
  nh.getParam("/camera_mv/GammaEnable", gamma_enable);  
  nh.getParam("/camera_mv/Gamma", gamma);  
  nh.getParam("/camera_mv/GainAuto", gain_auto);  
  nh.getParam("/camera_mv/SaturationEnable", saturation_enable);  
  nh.getParam("/camera_mv/Saturation", saturation);  
  nh.getParam("/camera_mv/TriggerMode", trigger_mode);  
  nh.getParam("/camera_mv/TriggerSource", trigger_source);  
  nh.getParam("/camera_mv/LineSelector", line_selector);  
  
  // get the frame id parameter (define in the launch file) ---------
  nh.getParam("/hikrobot_mv/hikrobot_mv_ros_node/frame_id", frame_id);
  
  // init the image data size ---------------------------------------
  image_data_size = 8 * width * height;
  
  // init. the camera publisher ------------------------------------
  img_transport = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
  cam_pub = img_transport->advertiseCamera("image_raw", 1);  // set the name of published image ("image_raw") and the buffer size (1) 
  
  // set some parameter of camera_info ------------------------------
  camera_info = std::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh));  
  if (!camera_info->isCalibrated())
  {
    camera_info->setCameraName("Hikrobot_MV");
    sensor_msgs::CameraInfo cam_info_;
    cam_info_.width = width;
    cam_info_.height = height;
    camera_info->setCameraInfo(cam_info_);
  }
  
  ROS_INFO("START INIT. HIKROBOT MV CAMERA =========================");

  // get the list of mv devices -------------------------------------
  mv_res = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != mv_res) 
  {
    ROS_INFO("MV_CC_EnumDevices fail :: error_code [%x]", mv_res);
    exit = true;
  }
  
  // get the info of mv devices -------------------------------------
  if (stDeviceList.nDeviceNum > 0)
  {
	for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
      ROS_INFO("[device %d]:", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (NULL == pDeviceInfo)
      {
        break;
      } 
      
      HikrobotMVDriver::printDeviceInfo(pDeviceInfo);            
    }   
  } 
  else
  {
    ROS_INFO("Find No Devices");
    exit = true;
  }
  
  // select the device 0 and create handle ------------------------
  mv_res = MV_CC_CreateHandle(&ptr_mv_handle, stDeviceList.pDeviceInfo[index]);
  if (MV_OK != mv_res)
  {
    ROS_INFO("MV_CC_CreateHandle fail :: error_code [%x]", mv_res);
    exit = true;
  }
   
  // open device ----------------------------------------------------
  mv_res = MV_CC_OpenDevice(ptr_mv_handle);
  if (MV_OK != mv_res)
  {
    ROS_INFO("MV_CC_OpenDevice fail :: error_code [%x]", mv_res);
    exit = true;
  }
  
  // Detection network optimal package size (It only works for the GigE camera)
  if (stDeviceList.pDeviceInfo[index]->nTLayerType == MV_GIGE_DEVICE)
  {
    int int_PacketSize = MV_CC_GetOptimalPacketSize(ptr_mv_handle);
            
    if (int_PacketSize > 0)
    {
      mv_res = MV_CC_SetIntValue(ptr_mv_handle,"GevSCPSPacketSize",int_PacketSize);
      if(mv_res != MV_OK)
      {
        ROS_INFO("Warning: Set Packet Size fail :: error_code [0x%x]", mv_res);
      }
    }
    else
    {
      ROS_INFO("Warning: Get Packet Size fail :: error_code [0x%x]", int_PacketSize);
    }
  }
  
  // set the properties ---------------------------------------------  
  ROS_INFO("SET HIKROBOT PARAMETERS ================================");
  
  this->setParameter(HIK_MV_PARAM_WIDTH, width);
  this->setParameter(HIK_MV_PARAM_HEIGHT, height);   
  this->setParameter(HIK_MV_PARAM_OFFSET_X, offset_x);
  this->setParameter(HIK_MV_PARAM_OFFSET_Y, offset_y);     
  this->setParameter(HIK_MV_PARAM_FRAME_RATE_ENABLE, frame_rate_enable);
  this->setParameter(HIK_MV_PARAM_FRAME_RATE, frame_rate);    
  // this->setParameter(HIK_MV_PARAM_BURSTFRAMECOUNT, burst_frame_count);   
  this->setParameter(HIK_MV_PARAM_EXPOSURE_AUTO, exposure_auto);  
  //this->setParameter(HIK_MV_PARAM_EXPOSURE_TIME, exposure_time);
  this->setParameter(HIK_MV_PARAM_GAMMA_ENABLE, gamma_enable);       
  //this->setParameter(HIK_MV_PARAM_GAMMA, gamma);
  this->setParameter(HIK_MV_PARAM_GAINAUTO, gain_auto);  
  //this->setParameter(HIK_MV_PARAM_SATURATION_ENABLE, saturation_enable);
  //this->setParameter(HIK_MV_PARAM_SATURATION, saturation);       
  this->setParameter(HIK_MV_PARAM_TRIGGER_MODE, trigger_mode);
  this->setParameter(HIK_MV_PARAM_TRIGGER_SOURCE, trigger_source);        
  this->setParameter(HIK_MV_PARAM_LINE_SELECTOR, line_selector);      
  this->setParameter(HIK_MV_PARAM_PIXEL_FORMAT, PixelType_Gvsp_Mono8);    
     
  // start grab image -----------------------------------------------
  mv_res = MV_CC_StartGrabbing(ptr_mv_handle);
  if (MV_OK != mv_res)
  {
     ROS_INFO("MV_CC_StartGrabbing fail :: error_code [%x]", mv_res);
     exit = true;
  }
  
  if (exit)
  {
    ROS_INFO("bad initialization of hikrobt mv camera");
    return;
  }
  else
  {
    capture_timer = nh.createTimer(ros::Duration(1.0 / frame_rate),
        boost::bind(&HikrobotMVDriver::captureAndPublish, this, _1));
  }
}

/**
  @fn bool HikrobotMVDriver::printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    print the device information on the console
  @param pstMVDevInfo pointer on the device information
  @return false or true    
*/
bool HikrobotMVDriver::printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
  // check if the pointer is not NULL -------------------------------
  if (NULL == pstMVDevInfo)
  {
    ROS_INFO("The Pointer of pstMVDevInfo is NULL");
    return false;
  }
  
  // device is connected via ethernet -------------------------------  
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
  {
    int ip1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int ip2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int ip3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int ip4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    // print current ip and user defined name
    ROS_INFO("Device Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    ROS_INFO("CurrentIp: %d.%d.%d.%d" , ip1, ip2, ip3, ip4);
    ROS_INFO("UserDefinedName: %s" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
   }
   // device is connected via usb ----------------------------------- 
   else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
   {
     ROS_INFO("Device Model Name: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
     ROS_INFO("UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
   }
   // device is not supported ---------------------------------------   
   else
   {
     ROS_INFO("Device is not support");
   }

   return true;
}

/**
  @fn bool HikrobotMVDriver::setParameter(HikMvParamlist hik_mv_param, double value)
    set the camera parameters
   @param hik_mv_param is one parameter of the HikMvParamlist
   @param value is the parameter value to set
  @return false or true    
*/
bool HikrobotMVDriver::setParameter(HikMvParamlist hik_mv_param, double value)
{
  bool ret_val = true;	
  int mv_res;		
	
  // set property depending of the type -----------------------------	
  switch(TAB_HIK_MV_SET_PARAM[hik_mv_param].type)
  {
	  case HIK_MV_TYPE_PARAM_BOOL:
	    mv_res = MV_CC_SetBoolValue(ptr_mv_handle, TAB_HIK_MV_SET_PARAM[hik_mv_param].str_key, value);
        break;
        
      case HIK_MV_TYPE_PARAM_INT:
	    mv_res = MV_CC_SetIntValue(ptr_mv_handle, TAB_HIK_MV_SET_PARAM[hik_mv_param].str_key, value);
        break;           
      
      case HIK_MV_TYPE_PARAM_FLOAT:
	    mv_res = MV_CC_SetFloatValue(ptr_mv_handle, TAB_HIK_MV_SET_PARAM[hik_mv_param].str_key, value);
        break;   
        
      case HIK_MV_TYPE_PARAM_ENUM:
	    mv_res = MV_CC_SetEnumValue(ptr_mv_handle, TAB_HIK_MV_SET_PARAM[hik_mv_param].str_key, value);
        break;  
        
      default:
        ret_val = false;
        break;               
  }
  
  // display the result ---------------------------------------------
  if(MV_OK == mv_res)
  {
    ROS_INFO("%s has been set to %.1f", TAB_HIK_MV_SET_PARAM[hik_mv_param].str_key, value);
  }
  else
  {
    ROS_INFO("%s failed :: value = %.1f :: error_code [%x]", TAB_HIK_MV_SET_PARAM[hik_mv_param].str_key, value, mv_res);	
    ret_val = false;     
  }	    
  
  return ret_val;
}

/**
  @fn bool HikrobotMVDriver::closeCamera()
    close camera
  @return false or true    
*/
bool HikrobotMVDriver::closeCamera()
{
  int ret_val = true;	
  int mv_res = MV_OK;	
	
  // end grab image -------------------------------------------------
  mv_res = MV_CC_StopGrabbing(ptr_mv_handle);
  if (MV_OK != mv_res)
  {
    ROS_INFO("MV_CC_StopGrabbing fail :: error_code [%x]", mv_res);
    ret_val = false;
  }

  // close device ---------------------------------------------------
  if(ret_val)
  {
    mv_res = MV_CC_CloseDevice(ptr_mv_handle);
    if (MV_OK != mv_res)
    {
      ROS_INFO("MV_CC_CloseDevice fail :: error_code [%x]", mv_res);
    }
  }  

  // destroy handle
  if(ret_val)
  {
    mv_res = MV_CC_DestroyHandle(ptr_mv_handle);
    if (MV_OK != mv_res)
    {
      ROS_INFO("MV_CC_DestroyHandle fail :: error_code [%x]", mv_res);
    }
  }  	
		
  return ret_val;
}

/**
  @fn void HikrobotMVDriver::captureAndPublish(const ros::TimerEvent& evt)
    capture one frame and publish it via the camera publisher
  @param TimerEvent    
*/
void HikrobotMVDriver::captureAndPublish(const ros::TimerEvent& evt)
{
  int mv_res = MV_OK;
  
  // allocate buffer for the raw image
  unsigned char *ptr_raw_buffer = (unsigned char *)malloc(sizeof(unsigned char) * image_data_size);
  
  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

  sensor_msgs::CameraInfoPtr ptr_cam_info(new sensor_msgs::CameraInfo(camera_info->getCameraInfo())); 
  ptr_cam_info->header.frame_id = frame_id;
  
  // get one frame --------------------------------------------------
  mv_res = MV_CC_GetOneFrameTimeout(ptr_mv_handle, ptr_raw_buffer, image_data_size, &stImageInfo, 1000);
  if (mv_res == MV_OK)
  {
    ROS_DEBUG("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
  }
  else
  {
    ROS_INFO("No data :: error_code [%x]", mv_res);
  }
   
  // publish image --------------------------------------------------
  raw_frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, ptr_raw_buffer); // other format: CV_8UC3 with PixelType_Gvsp_BGR8_Packed
  cv_img.image = raw_frame;
  cv_img.header.stamp = ros::Time::now();
  cv_img.header.frame_id = frame_id;
  cv_img.encoding = sensor_msgs::image_encodings::MONO8; // BGR8 if CV_8UC3
  pub_image = cv_img.toImageMsg();

  ptr_cam_info->header.stamp = pub_image->header.stamp;
  cam_pub.publish(pub_image, ptr_cam_info);

  free(ptr_raw_buffer);
}
