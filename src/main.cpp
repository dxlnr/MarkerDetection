/**
  * Author: Daniel Illner -- email: ge98riy@mytum.de or illner.d@protonmail.com
  *
  */
  
#include <iostream>

#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "findmarker.h"


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    cv::Mat rgb_img = cv_bridge::toCvShare(msg) -> image;

    cv::Mat grayScale;
		cv::Mat filtered = rgb_img.clone();

    FindMarker marker(filtered);
    marker.detectContours();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


/**
 * @function main
 */
int main(int argc, char** argv)
{

	ros::init(argc, argv, "marker_detection");

	ros::NodeHandle nh;
	//Listener listener;
	cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::spin();


}
