/*
	Author: Daniel Illner -- email: ge98riy@mytum.de or illner.d@protonmail.com
*/
#include <iostream>

#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#define THICKNESS_VALUE 4

// List of points
typedef std::vector<cv::Point> contour_typ;
// List of contours
typedef std::vector<contour_typ> contour_vector_typ;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    cv::Mat rgb_img = cv_bridge::toCvShare(msg) -> image;

    cv::Mat grayScale;
		cv::Mat filtered = rgb_img.clone();

		int slider_value = 170;

		/// Converting to grayscale
		cv::cvtColor(filtered, grayScale, cv::COLOR_BGR2GRAY);
		///Thresholding and converting to binary
		cv::threshold(grayScale, grayScale, slider_value, 255, cv::THRESH_BINARY);
		/// Finding contours in the image.
		contour_vector_typ contours;
		cv::findContours(grayScale, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

		for (size_t k = 0; k < contours.size(); k++)
		{

			// -------------------------------------------------
			contour_typ approx_contour;
			// Simplifying of the contour with the Ramer-Douglas-Peuker Algorithm
			cv::approxPolyDP(contours[k], approx_contour, arcLength(contours[k], true) * 0.02, true);

			cv::Scalar QUADRILATERAL_COLOR(0, 0, 255);
			cv::Scalar colour;

			cv::Rect r = cv::boundingRect(approx_contour);

			if (approx_contour.size() == 4) {
				colour = QUADRILATERAL_COLOR;
			}
			else {
				continue;
			}

			if (r.height < 20 || r.width < 20 || r.width > filtered.cols - 10 || r.height > filtered.rows - 10) {
				continue;
			}

			cv::polylines(filtered, approx_contour, true, colour, THICKNESS_VALUE);

      // --- Process Corners ---
			for (size_t i = 0; i < approx_contour.size(); i++) {
				// Iterate through every line of every rectangle and try to find the corner points.
				cv::circle(filtered, approx_contour[i], 3, CV_RGB(255, 255, 0), -1);

				double dx = ((double)approx_contour[(i + 1) % 4].x - (double)approx_contour[i].x) / 7.0;
				double dy = ((double)approx_contour[(i + 1) % 4].y - (double)approx_contour[i].y) / 7.0;


				for (size_t j = 0; j < 7; j++){

					// Position calculation
					double px = (double)approx_contour[i].x + (double)j * dx;
					double py = (double)approx_contour[i].y + (double)j * dy;

					cv::Point p;
					p.x = (int)px;
					p.y = (int)py;
					cv::circle(filtered, p, 2, CV_RGB(0, 255, 255), -1);
        }
      }
    }
    cv::imshow("view", filtered);
    cv::waitKey(30);
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
