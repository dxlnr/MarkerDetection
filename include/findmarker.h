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
#include <std_msgs/Float64MultiArray.h>


struct Stripe {
	int stripeLength;
	int nStop;
	int nStart;
	cv::Point2f stripeVecX;
	cv::Point2f stripeVecY;
};


class FindMarker {

  public:
  	FindMarker(std::vector<cv::Mat> image, std::vector<cv::Mat> camMatrix, std::vector<cv::Mat> distCoeffs, cv::Ptr<cv::aruco::DetectorParameters> params,
	  		   std::vector<std::vector<int>> img_size, std::vector<bool> haveCamInfo);
    ~FindMarker();

    void detectContours();
	void detectContoursAdvanced();

	marker_detection::reference ref;
	std_msgs::Float64MultiArray transfm_msg;

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

    cv::Mat _gige_image;
	cv::Mat _usb_image;
	std::vector<cv::Mat> cameraMatrix;
    std::vector<cv::Mat> distortionCoeffs;
	std::vector<cv::Vec3d> rvecs;
	std::vector<cv::Vec3d> tvecs;

	Stripe stripe;
	cv::Point2f fcorners[4];
	cv::Point2f set2Points[4];

	std::vector<std::vector<int>> img_size;
	std::vector<bool> haveCamInfo;
	
	cv::Mat rot_mat = cv::Mat::ones(cv::Size(3,3), CV_64F);
	cv::Mat transformation_mat = cv::Mat::zeros(cv::Size(4,4), CV_64F);

	const double markerSize = 0.2f;

	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

	// helper functions
	double dist2D(const cv::Point2f &p1, const cv::Point2f &p2);
	double dist3D(const cv::Point3f &p1, const cv::Point3f &p2);
	double dist3DtoCamera(const cv::Point3f &p);
	double calcMarkerArea(const std::vector<cv::Point2f> &pts);
	double calcAngleX(const cv::Point2f &p1, const cv::Point2f &p2);
	double calcAngleXAvg(std::vector<cv::Point2f> vpts1, std::vector<cv::Point2f> vpts2) ;
	cv::Mat GetWorldPoint(cv::Vec3d &rvecs);
	cv::Mat DoubleMatFromVec3b(cv::Vec3d &input);
};

#endif
