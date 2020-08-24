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

#include "findmarker.h"


class ReferenceFinder {
  public:

    ReferenceFinder(const ros::Publisher& pub) : reference_pub_(pub){
      haveCamInfo = false;
      cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
      distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);
      img_height = 0;
      img_width = 0;
      haveCamInfo = false;
      
      setDetectorParameters();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {

        //cv::Mat rgb_img = cv_bridge::toCvShare(msg) -> image;
        cv::Mat rgb_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) -> image;

        cv::Mat grayScale;
  		  cv::Mat filtered = rgb_img.clone();


        FindMarker marker(filtered, cameraMatrix, distortionCoeffs, params, img_height, img_width, haveCamInfo);
        marker.detectContours();

        reference_pub_.publish(marker.ref);

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
            img_height = msg->height;
            img_width = msg->width;
            //frameId = msg->header.frame_id;
        }
        else {
            ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
        }
    }

    void setDetectorParameters() {
      /**
        This function sets the detector parameters. 
        Firstly, the default is set and if wanted changes can be done here.
        */
      this -> params = cv::aruco::DetectorParameters::create();

      params -> doCornerRefinement = true;

    }

  private:
    ros::Publisher reference_pub_;

    bool haveCamInfo;
    int img_height;
    int img_width;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Ptr<cv::aruco::DetectorParameters> params;

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
  ros::Publisher pub = nh.advertise<marker_detection::reference>("coordinates", 10);
  ReferenceFinder ref(pub);

  ros::Subscriber caminfo_sub = nh.subscribe("/pylon_camera_node/camera_info", 1, &ReferenceFinder::camInfoCallback, &ref);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/pylon_camera_node/image_rect", 1, &ReferenceFinder::imageCallback, &ref);

  ros::spin();
  return 0;
}
