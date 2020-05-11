/**
  * listener.cpp
  *
  *  Created by: Daniel Illner on 12.04.2020
  */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "listener.h"

#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>

void msgCallback(const sensor_msgs::ImageConstPtr& msg);

Listener::Listener()
:
  test(NULL)
{
  ros::NodeHandle nh;

  int sqs = 1;
  std::cout << "Test 1" << std::endl;

  //ros::Subscriber camera_sub = nh.subscribe("/usb_cam/image_raw", 1, &Listener::msgCallback, this);

  //test = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/usb_cam/image_raw", sqs);
  //std::cout << typeid(test).name() << std::endl;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, msgCallback);
  //std::cout << typeid(sub).name() << std::endl;


  std::cout << "Test 2" << std::endl;

  //std::cout << typeid(camera_sub).name() << std::endl;

  //Listener::msgCallback(camera_sub);
  /*


  int sqs = 1;
  //ros::NodeHandle nh;
  tflistener_ = new tf::TransformListener(nh);

  image_transport::ImageTransport it(nh);

  //image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, Listener::rgbCallback);

  std::cout << "Test 2" << std::endl;

  //test = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/usb_cam/image_raw", sqs);

  std::string image;
  ros::param::get( "/usb_cam/image_raw", image);
  std::cout << "Test 3" << std::endl;
  cv::Mat src;
  src = cv::imread(image, 1 );
  std::cout << "Test 4" << std::endl;

  std::cout << "matrix = " << std::endl << " "  << src << std::endl << std::endl;

  const std::string source_window = "Source";
	cv::namedWindow(source_window, cv::WINDOW_NORMAL);
  std::cout << "Test 5" << std::endl;
	cv::imshow(source_window, src);
  std::cout << "Test 6" << std::endl;


  //visual_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/camera/rgb/image_color", sqs);
  //depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/camera/depth/image", sqs);
  //cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "/camera/depth_registered/points", sqs);

  //sensor_syn_ = new message_filters::Synchronizer<KinectSyncPolicy>(KinectSyncPolicy(sqs),  *visual_sub_);
  //sensor_syn_->registerCallback(boost::bind(&Listener::kinectCallback, this, _1));

  ROS_INFO_STREAM_NAMED("Listener", "is listening to " << "/camera");
  */
  cv::destroyWindow("view");

}

Listener::~Listener()
{
  delete tflistener_;
}


void Listener::visualization(cv::Mat rgb_image)
{
  for (;;){
    cv::imshow("RGB Frame", rgb_image);
    if(cv::waitKey(30) >= 0) break;
  }

}

void Listener::kinectCallback (const sensor_msgs::ImageConstPtr& rgb_img_msg,
                               const sensor_msgs::ImageConstPtr& depth_img_msg)
{
  cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_img_msg)-> image;
  //cv::Mat rgb_img = rotate(rgb_img_org, 180.0);

  //cv::Mat depth_img_org = cv_bridge::toCvCopy(depth_img_msg)->image;
  //cv::Mat depth_img = rotate(depth_img_org, 180.0);


  //pointcloudprep (point_cloud);
  /*
  ROS_INFO ("Nodeconstruction");
  Node* node_ptr = new Node(rgb_img);  //Node construction
  ROS_INFO ("successful");
  */
  visual_img_ = rgb_img.clone();

  visualization(visual_img_);

}


void Listener::rgbCallback (const sensor_msgs::ImageConstPtr& rgb_img_msg)
{
  /**
    Perform the rgb image callback.
    */
  std::cout << "Test rgbCallback" << std::endl;
  try
  {
    //cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_img_msg)-> image;
    cv::Mat rgb_img = cv_bridge::toCvShare(rgb_img_msg)-> image;
    for (;;){
      cv::imshow("view", rgb_img);
      if(cv::waitKey(30) >= 0) break;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rgb_img_msg-> encoding.c_str());
  }
}

void msgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat rgb_img = cv_bridge::toCvShare(msg) -> image;
  cv::Mat visual_img_ = rgb_img.clone();
  try
  {
    cv::imshow("view",visual_img_);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg-> encoding.c_str());
  }
  ///ROS_INFO("height = %d, width = %d",msg -> height, msg -> width);
}
