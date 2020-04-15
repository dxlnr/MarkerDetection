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




Listener::Listener()
:
  sensor_syn_(NULL),
  visual_sub_(NULL),
  depth_sub_(NULL)
{
  int sqs = 3;
  ros::NodeHandle nh;
  tflistener_ = new tf::TransformListener(nh);


  visual_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/camera/rgb/image_color", sqs);
  depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/camera/depth/image", sqs);
  //cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "/camera/depth_registered/points", sqs);

  //sensor_syn_ = new message_filters::Synchronizer<KinectSyncPolicy>(KinectSyncPolicy(sqs),  *visual_sub_);
  //sensor_syn_->registerCallback(boost::bind(&Listener::kinectCallback, this, _1));
  ROS_INFO_STREAM_NAMED("Listener", "is listening to " << "/camera");


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
