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
      haveCamInfo = false;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {

        cv::Mat rgb_img = cv_bridge::toCvShare(msg) -> image;

        cv::Mat grayScale;
  		  cv::Mat filtered = rgb_img.clone();

        FindMarker marker(filtered, cameraMatrix, distortionCoeffs);
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

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
        /**
          Subscribing the camera info, created by calibration process.
          */
        if (haveCamInfo) {
            return;
        }

        if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
            for (int i=0; i<3; i++) {
                for (int j=0; j<3; j++) {
                    cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
                }
            }

            for (int i=0; i<5; i++) {
                distortionCoeffs.at<double>(0,i) = msg->D[i];
            }

            haveCamInfo = true;
            //frameId = msg->header.frame_id;
        }
        else {
            ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
        }
    }

  private:
    ros::Publisher reference_pub_;

    bool haveCamInfo;
    cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

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

  ros::Subscriber caminfo_sub = nh.subscribe("/usb_cam/camera_info", 1, &ReferenceFinder::camInfoCallback, &ref);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &ReferenceFinder::imageCallback, &ref);

  ros::spin();
  return 0;
}
