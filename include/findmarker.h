/**
  * listener.cpp
  *
  *  Created by: Daniel Illner on 10.05.2020
  */

#ifndef FINDMARKER_H
#define FINDMARKER_H

#include <iostream>
#include <ros/ros.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"


struct Stripe {
	int stripeLength;
	int nStop;
	int nStart;
	cv::Point2f stripeVecX;
	cv::Point2f stripeVecY;
};


class FindMarker {

  public:
    FindMarker(cv::Mat image);
    ~FindMarker();

    void detectContours();

  private:

    int subpixSampleSafe(const cv::Mat& pSrc, const cv::Point2f& p);
    cv::Mat calculate_Stripe(double dx, double dy, Stripe & st);

		void computeIntersection();
		void setReferenzPoints()

    cv::Mat _image;
    cv::Mat grayScale;
		Stripe stripe;
		cv::Point2f corners[4];
		cv::Point2f set2Points[4];

		float lineParams[16]; // Direction vector (x0,y0) and contained point (x1,y1) -> For each line -> 4x4 = 16

    // List of points
    typedef std::vector<cv::Point> contour_typ;
    // List of contours
    typedef std::vector<contour_typ> contour_vector_typ;

    int thickness_value = 4;
    int threshold_value = 170;

};

#endif
