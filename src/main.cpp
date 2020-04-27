/*
	Author: Daniel Illner -- email: ge98riy@mytum.de or illner.d@protonmail.com
*/
#include <iostream>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "helper.h"

#include <pcl_ros/point_cloud.h>

#define THICKNESS_VALUE 4

struct Stripe {
	int stripeLength;
	int nStop;
	int nStart;
	cv::Point2f stripeVecX;
	cv::Point2f stripeVecY;
};


// List of points
typedef std::vector<cv::Point> contour_typ;
// List of contours
typedef std::vector<contour_typ> contour_vector_typ;



int subpixSampleSafe(const cv::Mat& pSrc, const cv::Point2f& p) {
	/**
		Starting from the subdivision point, you can defie two vectors with a length of
		one pixel, one in the direction of the original edge and an orthogonal one. For each of
		the pixels in your subimage, use this function to get the correct value from
		the original image:
		*/
	int fx = int(floorf(p.x));
	int fy = int(floorf(p.y));

	if (fx < 0 || fx >= pSrc.cols - 1 || fy < 0 || fy >= pSrc.rows - 1){
		return 127;
	}

	int px = int(256 * (p.x - floorf(p.x)));
	int py = int(256 * (p.y - floorf(p.y)));

	unsigned char* i = (unsigned char*)((pSrc.data + fy * pSrc.step) + fx);

	int a = i[0] + ((px * (i[1] - i[0])) >> 8);
	i += pSrc.step;
	int b = i[0] + ((px * (i[1] - i[0])) >> 8);

	return a + ((py * (b - a)) >> 8);
}

cv::Mat calculate_Stripe(double dx, double dy, Stripe & st) {

	st.stripeLength = (int)(0.8 * sqrt(dx * dx + dy * dy));

	if (st.stripeLength < 5){
		st.stripeLength = 5;
	}

	st.nStop = st.stripeLength >> 1;
	st.nStart = -st.nStop;

	cv::Size stripeSize;
	stripeSize.width = 3;
	stripeSize.height = st.stripeLength;


	// Normalized direction vector
	st.stripeVecX.x = dx / sqrt(dx * dx + dy * dy);
	st.stripeVecX.y = dy / sqrt(dx * dx + dy * dy);

	// Normalized perpendicular vector
	st.stripeVecY.x = st.stripeVecX.y;
	st.stripeVecY.y = -st.stripeVecX.x;

	return cv::Mat(stripeSize, CV_8UC1);
}


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
  cv::namedWindow(windowname);

	cv::Mat filtered;

	// Default resolution of the frame is obtained.The default resolution is system dependent.
	//int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	//int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	//cv::VideoWriter videoObject("../data/videowithmarkers.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height));

	bool isFirstStripe = true;

	while (cap.read(frame))
	{
		//cap >> frame;
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

				Stripe stripe;
				cv::Mat iplStripe = calculate_Stripe(dx, dy, stripe);

				for (size_t j = 0; j < 7; j++){

					// Position calculation
					double px = (double)approx_contour[i].x + (double)j * dx;
					double py = (double)approx_contour[i].y + (double)j * dy;

					cv::Point p;
					p.x = (int)px;
					p.y = (int)py;
					cv::circle(filtered, p, 2, CV_RGB(0, 255, 255), -1);

					// Compute Stripes of every point on one side of the rectangle.
					for (int m = -1; m <= 1; m++){
						for (int n = stripe.nStart; n <= stripe.nStop; n++){
							cv::Point2f subPixel;
							subPixel.x = (double)p.x + ((double)m * stripe.stripeVecX.x) + ((double)n * stripe.stripeVecY.x);
							subPixel.y = (double)p.y + ((double)m * stripe.stripeVecX.y) + ((double)n * stripe.stripeVecY.y);

							int pixelInt = subpixSampleSafe(grayScale, subPixel);
							// Converte from index to pixel coordinate.
							int w = m + 1;
							int h = n + (stripe.stripeLength >> 1);
							iplStripe.at<uchar>(h,w) = (uchar)pixelInt;

							/*
							cv::Point p2;
							p2.x = (int)subPixel.x;
							p2.y = (int)subPixel.y;

							if (isFirstStripe)
								circle(filtered, p2, 1, CV_RGB(255, 0, 255), -1);
							else
								circle(filtered, p2, 1, CV_RGB(0, 255, 255), -1);
							*/
						}
					}

					// Apply Sobel filter.

					std::vector<double> sobelValues(stripe.stripeLength - 2.);

					for (int l = 1; l < (stripe.stripeLength - 1); l++) {
						// Take the intensity value from the stripe
						unsigned char* stripePtr = &(iplStripe.at<uchar>(l - 1, 0));

						// Calculation of the gradient with the sobel for the first row
						double gr1 = -stripePtr[0] - 2. * stripePtr[1] - stripePtr[2];

						// Jump two lines
						stripePtr += 2 * iplStripe.step;

						double gr3 = stripePtr[0] + 2. * stripePtr[1] + stripePtr[2];

						unsigned int ti = l - 1;
						sobelValues[ti] = gr1 + gr3;
					}

					double maxIntensity = -1;
					int maxIntensityIndex = 0;

					// Finding the max value
					for (int o = 0; o < stripe.stripeLength - 2; o++) {
						if (sobelValues[o] > maxIntensity) {
							maxIntensity = sobelValues[o];
							maxIntensityIndex = o;
						}
					}

					double y0, y1, y2;
					unsigned int max1 = maxIntensityIndex - 1;
					unsigned int max2 = maxIntensityIndex + 1;

					// If the index is at the border we are out of the stripe, than we will take 0
					y0 = (maxIntensityIndex <= 0) ? 0 : sobelValues[max1];
					y1 = sobelValues[maxIntensityIndex];
					y2 = (maxIntensityIndex >= stripe.stripeLength - 3) ? 0 : sobelValues[max2];

					// Formula for calculating the x-coordinate of the vertex of a parabola, given 3 points with equal distances
					// (xv means the x value of the vertex, d the distance between the points):
					// xv = x1 + (d / 2) * (y2 - y0)/(2*y1 - y0 - y2)

					// d = 1 because of the normalization
					double pos = (y2 - y0) / (4 * y1 - 2 * y0 - 2 * y2);

					// Check if there is a solution to the calculation
					if (pos != pos) {
						continue;
					}

					// Exact point with subpixel accuracy
					cv::Point2d edgeCenter;

					int maxIndexShift = maxIntensityIndex - (stripe.stripeLength >> 1);

					// Shift the original edgepoint accordingly
					edgeCenter.x = (double)p.x + (((double)maxIndexShift + pos) * stripe.stripeVecY.x);
					edgeCenter.y = (double)p.y + (((double)maxIndexShift + pos) * stripe.stripeVecY.y);

					// Highlight the subpixel with blue color
					cv::circle(filtered, edgeCenter, 2, CV_RGB(0, 0, 255), -1);
				}
			}
		}
		//videoObject.write(filtered);

		cv::resize(filtered, filtered, cv::Size(2560, 1920));
		cv::imshow(windowname, filtered);
		isFirstStripe = true;

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
