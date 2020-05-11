/**
  * listener.h
  *
  *  Created by: Daniel Illner on 12.04.2020
  */

#ifndef LISTENER_H
#define LISTENER_H
#include "ros/ros.h"
#include <rosbag/bag.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tf/transform_listener.h>

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> KinectSyncPolicy;


class Listener {

  public:
    Listener();
    ~Listener();

    void kinectCallback (const sensor_msgs::ImageConstPtr& visual_img,
                         const sensor_msgs::ImageConstPtr& depth_img);

    void rgbCallback (const sensor_msgs::ImageConstPtr& visual_img);

    //void msgCallback (const sensor_msgs::ImageConstPtr& msg);

  protected:
    void visualization(cv::Mat rgb_image);

    cv::Mat visual_img_; // for visualization purpose.

    message_filters::Synchronizer<KinectSyncPolicy>* sensor_syn_;
    message_filters::Subscriber<sensor_msgs::Image> *test;
    message_filters::Subscriber<sensor_msgs::Image> *visual_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    tf::TransformListener* tflistener_;

};

#endif
