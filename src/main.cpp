/**
  * Author: Daniel Illner -- email: ge98riy@mytum.de or illner.d@protonmail.com
  *
  */

#include <iostream>

#include <stdio.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

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

    void oneImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      ROS_INFO("Subscribing to one camera. Pylon Camera");
      try
      {
        cv::Mat rgb_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) -> image;

        cv::Mat grayScale;
  		  cv::Mat filtered = rgb_img.clone();

        FindMarker marker(filtered, cameraMatrix, distortionCoeffs, params, img_height, img_width, haveCamInfo);
        marker.detectContours();

        //reference_pub_.publish(marker.ref);
	      reference_pub_.publish(marker.transfm_msg);

      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }

    void twoImagesCallback(const sensor_msgs::ImageConstPtr& msgleft, const sensor_msgs::ImageConstPtr& msgright)
    {
      ROS_INFO("Synchronization successful. Subscribing to two cameras.");
      try
      {
        cv::Mat img_left = cv_bridge::toCvCopy(msgleft, sensor_msgs::image_encodings::BGR8) -> image;
        cv::Mat img_right = cv_bridge::toCvCopy(msgright, sensor_msgs::image_encodings::BGR8) -> image;

  		  cv::Mat filtered_left = img_left.clone();
        cv::Mat filtered_right = img_right.clone();

        // TODO: REGISTER THE TWO IMAGES OF THE CAMERAS AND PERFORM 3D STUFF.

        cv::namedWindow("Pylon View", cv::WINDOW_NORMAL);
        cv::imshow("Pylon View", img_left);
        cv::namedWindow("USB View", cv::WINDOW_NORMAL);
        cv::imshow("USB View", img_right);
        cv::waitKey(10);

      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Either could not convert image from '%s' to 'bgr8'.", msgleft->encoding.c_str());
        ROS_ERROR("or could not convert image from '%s' to 'bgr8'.", msgright->encoding.c_str());
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
            ros::Time tstamp = msg->header.stamp;
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
      params -> cornerRefinementMaxIterations = 100;
      params -> cornerRefinementMinAccuracy = 0.05;

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
	ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  // Read in the parameters how many cameras you want to use.
  std::string param;
  nh.getParam("param", param);
  ROS_INFO("Got parameter : %s", param.c_str());

  // Publisher section
  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("coordinates", 10);
  ReferenceFinder ref(pub);

  if(param.compare("onecam") == 0)
  {
    ros::Subscriber caminfo_sub = nh.subscribe("/pylon_camera_node/camera_info", 1, &ReferenceFinder::camInfoCallback, &ref);
    image_transport::Subscriber sub = it.subscribe("/pylon_camera_node/image_rect", 1, &ReferenceFinder::oneImageCallback, &ref);

    ros::spin();
  }
  else if(param.compare("twocam") == 0)
  {
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/pylon_camera_node/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> usb_image_sub(nh, "/usb_cam/image_raw", 1);

    //ros::Subscriber caminfo_sub = nh.subscribe("/pylon_camera_node/camera_info", 1, &ReferenceFinder::camInfoCallback, &ref);

    //TODO: ENABLE A SECOND CAMERA INFO OPTION TO HAVE THE CALIBRATION INOF AVAILABLE FOR BOTH.

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), image_sub, usb_image_sub);
    sync.registerCallback(boost::bind(&ReferenceFinder::twoImagesCallback, &ref, _1, _2));

    ros::spin();
  }
  else
  {
    std::cout << "System is running the default option with one camera! -> " << "You can specify how many cameras should be used." << std::endl;
    std::cout << "Enter either: rosrun marker_detection marker_detction _param:=onecam" << std::endl;
    std::cout << "or: rosrun marker_detection marker_detction _param:=twocam" << std::endl;

    ros::Subscriber caminfo_sub = nh.subscribe("/pylon_camera_node/camera_info", 1, &ReferenceFinder::camInfoCallback, &ref);
    image_transport::Subscriber sub = it.subscribe("/pylon_camera_node/image_rect", 1, &ReferenceFinder::oneImageCallback, &ref);

    ros::spin();
  }

  return 0;
}
