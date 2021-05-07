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

#ifndef HIKROBOT_MV_ROS_HIKROBOT_MV_DRIVER_H
#define HIKROBOT_MV_ROS_HIKROBOT_MV_DRIVER_H

// C++ Includes
#include <string>

// OpenCV Includes
#include <opencv2/opencv.hpp>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// Hikrobot Includes
#include "hikrobot_mv_ros/MvCameraControl.h"

namespace hikrobot_mv_ros
{   
  /* enum ---------------------------------------------------------*/
  enum HikMvParamlist
  {
	// image size  
    HIK_MV_PARAM_WIDTH = 0, 	  
    HIK_MV_PARAM_HEIGHT,             	  
    HIK_MV_PARAM_OFFSET_X,     
    HIK_MV_PARAM_OFFSET_Y,  	
    // frame rate  
    HIK_MV_PARAM_FRAME_RATE_ENABLE,  
    HIK_MV_PARAM_FRAME_RATE,        
    HIK_MV_PARAM_BURST_FRAME_COUNT,  
    // exposure    
    HIK_MV_PARAM_EXPOSURE_AUTO,
    HIK_MV_PARAM_EXPOSURE_TIME, 
    // gamma 
    HIK_MV_PARAM_GAMMA_ENABLE,     
    HIK_MV_PARAM_GAMMA,          
    HIK_MV_PARAM_GAINAUTO,  
    // saturation      
    HIK_MV_PARAM_SATURATION_ENABLE, 
    HIK_MV_PARAM_SATURATION,     
    // trigger
    HIK_MV_PARAM_TRIGGER_MODE,    
    HIK_MV_PARAM_TRIGGER_SOURCE,   
    HIK_MV_PARAM_LINE_SELECTOR,
    // pixel
    HIK_MV_PARAM_PIXEL_FORMAT,   
    HIK_MV_PARAM_NB  // used as the size of the list
  };
   
  enum HikMvTypeParamlist
  {
    HIK_MV_TYPE_PARAM_BOOL = 0,
    HIK_MV_TYPE_PARAM_STRING,      
    HIK_MV_TYPE_PARAM_INT,
    HIK_MV_TYPE_PARAM_FLOAT,  
    HIK_MV_TYPE_PARAM_ENUM,     
    HIK_MV_TYPE_PARAM_NB // used as the size of the list
  };     

  /* structure ----------------------------------------------------*/
  typedef struct
  {
    int type;
    char str_key[32]; 
  } tHikMvSetParam;

  /* constantes ---------------------------------------------------*/
  const tHikMvSetParam TAB_HIK_MV_SET_PARAM[HIK_MV_PARAM_NB] =
  {
	// type                        str_key
    HIK_MV_TYPE_PARAM_INT,          "Width",                         // HIK_MV_PARAM_WIDTH   		
    HIK_MV_TYPE_PARAM_INT,          "Height",                        // HIK_MV_PARAM_HEIGHT          
    HIK_MV_TYPE_PARAM_INT,          "OffsetX",                       // HIK_MV_PARAM_OFFSET_X
    HIK_MV_TYPE_PARAM_INT,          "OffsetY",                       // HIK_MV_PARAM_OFFSET_Y    
    HIK_MV_TYPE_PARAM_BOOL,         "AcquisitionFrameRateEnable",  	 // HIK_MV_PARAM_FRAME_RATE_ENABLE
    HIK_MV_TYPE_PARAM_FLOAT,        "AcquisitionFrameRate",	         // HIK_MV_PARAM_FRAME_RATE 
    HIK_MV_TYPE_PARAM_INT,          "AcquisitionBurstFrameCount",	 // HIK_MV_PARAM_BURST_FRAME_COUNT        
    HIK_MV_TYPE_PARAM_ENUM,         "ExposureAuto",                  // HIK_MV_PARAM_EXPOSURE_AUTO  
    HIK_MV_TYPE_PARAM_FLOAT,        "ExposureTime",                  // HIK_MV_PARAM_EXPOSURE_TIME
    HIK_MV_TYPE_PARAM_BOOL,         "GammaEnable",                   // HIK_MV_PARAM_GAMMA_ENABLE
    HIK_MV_TYPE_PARAM_FLOAT,        "Gamma",                         // HIK_MV_PARAM_GAMMA
    HIK_MV_TYPE_PARAM_ENUM,         "GainAuto",                      // HIK_MV_PARAM_GAINAUTO
    HIK_MV_TYPE_PARAM_BOOL,         "SaturationEnable",              // HIK_MV_PARAM_SATURATION_ENABLE
    HIK_MV_TYPE_PARAM_INT,          "Saturation",                    // HIK_MV_PARAM_SATURATION
    HIK_MV_TYPE_PARAM_ENUM,         "TriggerMode",                   // HIK_MV_PARAM_TRIGGER_MODE
    HIK_MV_TYPE_PARAM_ENUM,         "TriggerSource",                 // HIK_MV_PARAM_TRIGGER_SOURCE
    HIK_MV_TYPE_PARAM_ENUM,         "LineSelector",                  // HIK_MV_PARAM_LINE_SELECTOR
    HIK_MV_TYPE_PARAM_ENUM,         "PixelFormat",                   // HIK_MV_PARAM_PIXEL_FORMAT    
  };

  /* class --------------------------------------------------------*/	  
  class HikrobotMVDriver : public nodelet::Nodelet
  {		 	
	public:
    /* public attributes -----------------------------*/	  
    /* public methods --------------------------------*/
	HikrobotMVDriver();
	~HikrobotMVDriver();

	private:
    /* private attributes ----------------------------*/	  
	ros::NodeHandle nh;
	ros::Timer capture_timer;                        // use to set the sampling rate of capture and publish	method
	  	
	std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info; 	// camera_info use to define some parameter (name, frame_id...) => TODO use it instead of ptr_cam_info...
	std::shared_ptr<image_transport::ImageTransport> img_transport;         // image transport use to set the advertisment of camera publisher
	  	
	cv::Mat raw_frame;                               // frame is the n-dimensional dense numerical with contains the raw image
	image_transport::CameraPublisher cam_pub;        // cam_pub manages the advertisements for publishing camera image
	cv_bridge::CvImage cv_img;                       // cv_img allows to configure the parameters of the published image
	sensor_msgs::ImagePtr pub_image;                 // published image (via the image transport class)
	int image_data_size;	                         // size of the raw data image	
	
	std::string frame_id;                            // frame_id use to identify the published data
		
	void* ptr_mv_handle = NULL;                      // handle used for the mv driver librairies
		
		
	// these parameters are setting in the yaml config file
	int width;
	int height;  
	int offset_x;
	int offset_y;  
	int frame_rate_enable;
	float frame_rate;
	int burst_frame_count;  
	int exposure_auto;    
	float exposure_time;    
	int gamma_enable;  
	float gamma;  
	int gain_auto;  
	int saturation_enable;  
	int saturation;  
	int trigger_mode;  
	int trigger_source;  
	int line_selector;  	

    /* private methods -------------------------------*/	  
	virtual void onInit();
	bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
	bool setParameter(HikMvParamlist hik_mv_param, double value);
	bool closeCamera();
	void captureAndPublish(const ros::TimerEvent& evt);
};

}  // namespace hikrobot_mv_ros

#endif  // HIKROBOT_MV_ROS_HIKROBOT_MV_DRIVER_H
