/*
	Author: Daniel Illner -- email: ge98riy@mytum.de or illner.d@protonmail.com
*/
#include <iostream>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "helper.h"

#define THICKNESS_VALUE 4


// List of points
typedef std::vector<cv::Point> contour_typ;
// List of contours
typedef std::vector<contour_typ> contour_vector_typ;




/**
 * @function main
 */
int main(int argc, char** argv)
{

	ros::init(argc, argv, "marker_detection");

	/// Read in a specific videoframe.
	helper hlp;
	std::string videopath = hlp.resolvePath("/src/marker_detection/data/templatevideo.MP4");
	//std::string videopath = hlp.resolvePath("/src/marker_detection/data/curtain_wall_movement_01.mp4");

	std::cout << videopath << std::endl;

	cv::Mat frame;
	cv::VideoCapture cap(videopath);

	if (!cap.isOpened())
	{
		std::cout << "No video frame captured." << std::endl;
		return -1;
	}
	std::string windowname = "Frame Capture";
	//cv::namedWindow(windowname, CV_WINDOW_FREERATIO);
  cv::namedWindow(windowname);

	cv::Mat filtered;
	int slider_value = 170;

	// Default resolution of the frame is obtained.The default resolution is system dependent.
	//int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	//int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	//cv::VideoWriter videoObject("../data/videowithmarkers.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height));


	while (cap.read(frame))
	{
		cap >> frame;
		/// Preparing the frame.
		cv::Mat grayScale;
		filtered = frame.clone();
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


		}
		//videoObject.write(filtered);

		cv::imshow(windowname, filtered);
		int key = cv::waitKey(10);
		if (key == 27)
			break;

	}

	cap.release();
	//videoObject.release();

	cv::destroyWindow(windowname);

	ros::spin();

	return 0;
}
