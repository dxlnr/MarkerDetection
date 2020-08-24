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
#include <cmath>

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"

#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>
#include <marker_detection/MarkerVertices.h>
#include <marker_detection/reference.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


struct Stripe {
	int stripeLength;
	int nStop;
	int nStart;
	cv::Point2f stripeVecX;
	cv::Point2f stripeVecY;
};


class FindMarker {

  public:
  	FindMarker(cv::Mat image, cv::Mat camMatrix, cv::Mat distCoeffs, cv::Ptr<cv::aruco::DetectorParameters> params,
	  		   int img_height, int img_width, bool haveCamInfo);
    ~FindMarker();

    void detectContours();

	marker_detection::reference ref;

  private:

    int subpixSampleSafe(const cv::Mat& pSrc, const cv::Point2f& p);
    cv::Mat calculate_Stripe(double dx, double dy, Stripe & st);

	void computeIntersection(float lineParams[16]);
	void setReferenzPoints();

	void estimateMarkerPose(const std::vector<int> &ids,
							const std::vector<std::vector<cv::Point2f>> &corners, float markerLength,
							const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
							std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, std::vector<double>& projErr);
	
	double getReprojectionError(const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f> &imagePoints,
                             	const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const cv::Vec3d &rvec, const cv::Vec3d &tvec);

	void setSingleReferenzPoints(float markerLength, std::vector<cv::Point3f> &c_point);

    cv::Mat _image;
    cv::Mat grayScale;
	cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
	std::vector<cv::Vec3d> rvecs;
	std::vector<cv::Vec3d> tvecs;

	Stripe stripe;
	cv::Point2f fcorners[4];
	cv::Point2f set2Points[4];

	
	// Important values
	int camera_resolution_x;
	int camera_resolution_y;
	bool haveCamInfo;

	const float markerSize = 0.1f;

	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

	double dist2D(const cv::Point2f &p1, const cv::Point2f &p2);
	double dist3D(const cv::Point3f &p1, const cv::Point3f &p2);
	double dist3DtoCamera(const cv::Point3f &p);
	double calcMarkerArea(const std::vector<cv::Point2f> &pts);
	double calcAngleX(const cv::Point2f &p1, const cv::Point2f &p2);
	double calcAngleXAvg(std::vector<cv::Point2f> vpts1, std::vector<cv::Point2f> vpts2) ;

};

#endif
