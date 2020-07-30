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
#include "opencv2/aruco.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include "findmarker.h"


class ReferenceFinder {
  public:

    ReferenceFinder(const ros::Publisher& pub) : reference_pub_(pub){

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {

        cv::Mat rgb_img = cv_bridge::toCvShare(msg) -> image;

        cv::Mat grayScale;
  		  cv::Mat filtered = rgb_img.clone();

        FindMarker marker(filtered);
        marker.detectContours();

        // Publish the found coordinates.
        if (marker.coordinates_msg.x != 0.0 && marker.coordinates_msg.y != 0.0 && marker.coordinates_msg.z != 0.0) {
          reference_pub_.publish(marker.coordinates_msg);
        }


      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
  private:
    ros::Publisher reference_pub_;
};


/**
 * @function main
 */
int main(int argc, char** argv)
{
  // Initialize the ROS node.
	ros::init(argc, argv, "marker_detection");
	ros::NodeHandle nh;

  // Publisher section
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("coordinates", 100);

  ReferenceFinder ref(pub);

  // Subscriber section
	cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &ReferenceFinder::imageCallback, &ref);

  ros::spin();
  return 0;
}
