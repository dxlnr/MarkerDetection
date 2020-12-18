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

			// First Camera parameter
			cameraMatrix[0] = cv::Mat::zeros(3, 3, CV_64F);
			distortionCoeffs[0] = cv::Mat::zeros(1, 5, CV_64F);
			haveCamInfo[0] = false;

			setDetectorParameters();

			// Second Camera parameter
			cameraMatrix[1] = cv::Mat::zeros(3, 3, CV_64F);
			distortionCoeffs[1] = cv::Mat::zeros(1, 5, CV_64F);
			haveCamInfo[1] = false;
		}

		void oneImageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			ROS_INFO("Subscribing to one camera. Pylon Camera");
			try
			{
				std::vector<cv::Mat> image = std::vector<cv::Mat>(2);
				cv::Mat rgb_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) -> image;
				image[0] = rgb_img.clone();
				image[1] = cv::Mat::zeros(cv::Size(img_size[1][0], img_size[1][1]), CV_64FC1);

				FindMarker marker(image, cameraMatrix, distortionCoeffs, params, img_size, haveCamInfo);
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
				std::vector<cv::Mat> image = std::vector<cv::Mat>(2);
				cv::Mat gige_img = cv_bridge::toCvCopy(msgleft, sensor_msgs::image_encodings::BGR8) -> image;
				cv::Mat usb_img = cv_bridge::toCvCopy(msgright, sensor_msgs::image_encodings::BGR8) -> image;

				image[0] = gige_img.clone();
				image[1] = usb_img.clone();

				// TODO: REGISTER THE TWO IMAGES OF THE CAMERAS AND PERFORM 3D STUFF.
				FindMarker marker(image, cameraMatrix, distortionCoeffs, params, img_size, haveCamInfo);
				marker.detectContoursAdvanced();

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

					TODO: IMPLEMENT ALSO FOR USB CAMERA.
					*/

				if (haveCamInfo[0]) {
						return;
				}

				if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
						for (int i=0; i<3; i++) {
								for (int j=0; j<3; j++) {
										cameraMatrix[0].at<double>(i, j) = msg->K[i*3+j];
								}
						}
						for (int i=0; i<5; i++) {
								distortionCoeffs[0].at<double>(0,i) = msg->D[i];
						}

						haveCamInfo[0] = true;
						img_size[0][0] = msg->height;
						img_size[0][1] = msg->width;
						//frameId = msg->header.frame_id;
						//ros::Time tstamp = msg->header.stamp;
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

		// First Camera (GIGe) parameter.
		std::vector<bool> haveCamInfo = std::vector<bool>(2);
		std::vector<std::vector<int>> img_size{2, std::vector<int>(2,0)};

		std::vector<cv::Mat> cameraMatrix = std::vector<cv::Mat>(2);
		std::vector<cv::Mat> distortionCoeffs = std::vector<cv::Mat>(2);
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
