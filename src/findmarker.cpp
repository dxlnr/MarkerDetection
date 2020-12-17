/**
  * findmarker.cpp
  *
  *  Created by: Daniel Illner on 10.05.2020
  */

#include "findmarker.h"
#include "poseEstimation.h"


FindMarker::FindMarker(std::vector<cv::Mat> image, std::vector<cv::Mat> camMatrix, std::vector<cv::Mat> distCoeffs, cv::Ptr<cv::aruco::DetectorParameters> params, 
                       std::vector<std::vector<int>> img_size, std::vector<bool> haveCamInfo) {
 
  this -> _gige_image = image[0];
  this -> _usb_image = image[1];

  this -> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

  //Check if camera is calibrated.
  this -> haveCamInfo = haveCamInfo;

  // Matrix that values camera intrinsics
  this -> cameraMatrix = camMatrix;

  // Matrix that values distortion coefficients
  this -> distortionCoeffs = distCoeffs;

  // Set the special parameters for detecting the aruco markers.
  this -> detectorParams = params;

  // Receive resolution.
  this -> img_size = img_size;
}

FindMarker::~FindMarker(){
}


void FindMarker::detectContours(){

  if (!haveCamInfo[0]) {
    ROS_ERROR("No camera intrinsics");
    return;
  }
  
  transfm_msg.data.resize(16);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> rejectedCandidates;

  cv::aruco::detectMarkers(_gige_image, dictionary, corners, ids, detectorParams, rejectedCandidates);

  for (size_t i=0; i<ids.size(); i++) {
	    
    marker_detection::MarkerVertices mv;
    mv.marker_id = ids[i];

    mv.x0 = corners[i][0].x;
    mv.y0 = corners[i][0].y;
    mv.x1 = corners[i][1].x;
    mv.y1 = corners[i][1].y;
    mv.x2 = corners[i][2].x;
    mv.y2 = corners[i][2].y;
    mv.x3 = corners[i][3].x;
    mv.y3 = corners[i][3].y;
  }

  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(_gige_image, corners, ids);
  }

  // Find pose estimation.
  std::vector <double> projError;
  estimateMarkerPose(ids, corners, markerSize, cameraMatrix[0], distortionCoeffs[0], rvecs, tvecs, projError);

  double objError = 0.0;

  for (size_t i=0; i < ids.size(); i++) {
	  
	  if (ids[i] == 10 || ids[i] == 50 || ids[i] == 100) {
	    cv::aruco::drawAxis(_gige_image, cameraMatrix[0], distortionCoeffs[0], rvecs[i], tvecs[i], 0.1);

	    ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
		      tvecs[i][0], tvecs[i][1], tvecs[i][2],
		      rvecs[i][0], rvecs[i][1], rvecs[i][2]);

      objError = (projError[i] / dist2D(corners[i][0], corners[i][2])) *
                     (norm(tvecs[i]) / markerSize);

      // System only uses one aruco maker that is ID 10.
      if (ids[i] == 10) {
        ref.tx = tvecs[i][0];
	      ref.ty = tvecs[i][1];
	      ref.tz = tvecs[i][2];

        ref.rx = rvecs[i][0];
        ref.ry = rvecs[i][1];
        ref.rz = rvecs[i][2];

        cv::Mat R = GetWorldPoint(rvecs[i]);
      	cv::Mat rot_mat = R.t();
	
	      transfm_msg.data[0] = rot_mat.at<double>(0,0); 
        transfm_msg.data[1] = rot_mat.at<double>(0,1); 
        transfm_msg.data[2] = rot_mat.at<double>(0,2);
        transfm_msg.data[3] = tvecs[i][0];
        transfm_msg.data[4] = rot_mat.at<double>(1,0);
        transfm_msg.data[5] = rot_mat.at<double>(1,1);
        transfm_msg.data[6] = rot_mat.at<double>(1,2);
        transfm_msg.data[7] = tvecs[i][1];
        transfm_msg.data[8] = rot_mat.at<double>(2,0);
        transfm_msg.data[9] = rot_mat.at<double>(2,1);
        transfm_msg.data[10] = rot_mat.at<double>(2,2);
        transfm_msg.data[11] = tvecs[i][2];
	      transfm_msg.data[12] = 0.0;
	      transfm_msg.data[13] = 0.0;
	      transfm_msg.data[14] = 0.0;
        transfm_msg.data[15] = 1.0;

      }
    }
  }

  // Putting the camera output on screen.
  cv::namedWindow("Pylon View", cv::WINDOW_NORMAL);
  //cv::resize(_gige_image,_gige_image, cv::Size(640, 480));
  cv::imshow("Pylon View", _gige_image);
  cv::waitKey(10);

}


void FindMarker::detectContoursAdvanced(){
  /**
    * This function detects markers but uses two cameras as input.
    * TODO: THINK ABOUT REGISTERING. AND FURTHER DEVELOPMENTS.
    */
  if (!haveCamInfo[0] || !haveCamInfo[1]) {
    ROS_ERROR("No camera intrinsics");
    return;
  }
  
  transfm_msg.data.resize(16);

  std::vector<std::vector<int>> ids(2);
  std::vector<std::vector<std::vector<cv::Point2f>>> corners(2);
  std::vector<std::vector<std::vector<cv::Point2f>>> rejectedCandidates(2);

  cv::aruco::detectMarkers(_gige_image, dictionary, corners[0], ids[0], detectorParams, rejectedCandidates[0]);
  cv::aruco::detectMarkers(_usb_image, dictionary, corners[1], ids[1], detectorParams, rejectedCandidates[1]);

  if (ids[0].size() > 0) {
    cv::aruco::drawDetectedMarkers(_gige_image, corners[0], ids[0]);
  }
  if (ids[1].size() > 0) {
    cv::aruco::drawDetectedMarkers(_usb_image, corners[1], ids[1]);
  }
  
  cv::namedWindow("Pylon View", cv::WINDOW_NORMAL);
  cv::imshow("Pylon View", _gige_image);
  cv::namedWindow("USB View", cv::WINDOW_NORMAL);
  cv::imshow("USB View", _usb_image);
  cv::waitKey(10);
}


int FindMarker::subpixSampleSafe(const cv::Mat& pSrc, const cv::Point2f& p) {
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

cv::Mat FindMarker::calculate_Stripe(double dx, double dy, Stripe & st) {

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


void FindMarker::computeIntersection(float lineParams[16]) {
  /**
    Calculate the intersection points of both lines
    */
	for (int i = 0; i < 4; ++i) {
    // Go through the corners of the rectangle, 3 -> 0
    int j = (i + 1) % 4;

    double x0, x1, y0, y1, u0, u1, v0, v1;

    // We have to jump through the 4x4 matrix, meaning the next value for the wanted line is in the next row -> +4
    // We want to have the point first
    // g1 = (x0,y0) + a*(u0,v0) == g2 = (x1,y1) + b*(u1,v1)
    x0 = lineParams[i + 8]; y0 = lineParams[i + 12];
    x1 = lineParams[j + 8]; y1 = lineParams[j + 12];

    // Direction vector
    u0 = lineParams[i]; v0 = lineParams[i + 4];
    u1 = lineParams[j]; v1 = lineParams[j + 4];

    // Cramer's rule
    // 2 unknown a,b -> Equation system
    double a = x1 * u0 * v1 - y1 * u0 * u1 - x0 * u1 * v0 + y0 * u0 * u1;
    double b = -x0 * v0 * v1 + y0 * u0 * v1 + x1 * v0 * v1 - y1 * v0 * u1;

    // Calculate the cross product to check if both direction vectors are parallel -> = 0
    // c -> Determinant = 0 -> linear dependent -> the direction vectors are parallel -> No division with 0
    double c = v1 * u0 - v0 * u1;
    if (fabs(c) < 0.001) {
      continue;
    }

    // We have checked for parallelism of the direction vectors
    // -> Cramer's rule, now divide through the main determinant
    a /= c;
    b /= c;

    // Exact corner
    fcorners[i].x = a;
    fcorners[i].y = b;

    cv::Point pt;
    pt.x = (int)fcorners[i].x;
    pt.y = (int)fcorners[i].y;

    cv::circle(_gige_image, pt, 5, CV_RGB(0, 0, 255), -1);
  }
}

void FindMarker::setReferenzPoints(){
  /**
    The function sets the second set of 4 2D points.
    The points (−0.5, −0.5)(−0.5, 5.5)(5.5, 5.5)(5.5, −0.5) are used.
    */
  set2Points[0].x = -0.5;  set2Points[0].y = -0.5;
  set2Points[0].x = -0.5;  set2Points[0].y = 5.5;
  set2Points[0].x = 5.5;  set2Points[0].y = 5.5;
  set2Points[0].x = 5.5;  set2Points[0].y = -0.5;
}

void FindMarker::setSingleReferenzPoints(float markerLength, std::vector<cv::Point3f> &c_point){
  /**
    The function set and returns center points for a single marker using the length of a marker.
    :params markerLength: float number that gives the length of a detected marker in outside world (fixed).
    :params c_point: center point.
    */

  // setting coordinate system in the middle of the marker.
  c_point.clear();
  c_point.push_back(cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
  c_point.push_back(cv::Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
  c_point.push_back(cv::Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
  c_point.push_back(cv::Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}


void FindMarker::estimateMarkerPose(const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners, float markerLength,
                                    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                    std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, std::vector<double> &projErr){
  /**
    Function estimates the pose of a single Marker in regard to the camera. This is done for every marker found in image.
    :params ids: vector of the marker IDs found in the image.
    :params corners: vector that contains a corner point of each marker found in image.
    :params markerLength: float number that gives the length of a detected marker in outside world (fixed).
    :params cameraMatrix: camera matrix that is set by calibration.
    :params distCoeffs: distortion coefficients set by calibration.
    :params rvecs: rotation vector
    :params tvecs: translation vector
    */
  CV_Assert(markerLength > 0);

  int n_marker = (int)corners.size();
  std::vector<cv::Point3f> objPoints;
  rvecs.reserve(n_marker);
  tvecs.reserve(n_marker);
  projErr.reserve(n_marker);

  for (int i = 0; i < n_marker; i++) {
    setSingleReferenzPoints(markerLength, objPoints);
    cv::solvePnP(objPoints, corners[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);

    projErr[i] = getReprojectionError(objPoints, corners[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);
  }

}

double FindMarker::getReprojectionError(const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f> &imagePoints,
                                        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    /**
      The function estimates the reprojection error by using RMS.
      :params objPoints:
      :params imagePoints:
      :params cameraMatrix: camera matrix that is set by calibration.
      :params distCoeffs: distortion coefficients set by calibration.
      :params rvecs: rotation vector
      :params tvecs: translation vector
      */

    std::vector<cv::Point2f> projectedPoints;

    cv::projectPoints(objPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    double totalError = 0.0;
    for (unsigned int i=0; i < objPoints.size(); i++) {
        double error = dist2D(imagePoints[i], projectedPoints[i]);
        totalError += error * error;
    }

    return totalError / (double)objPoints.size();
}

double FindMarker::dist2D(const cv::Point2f &p1, const cv::Point2f &p2) {
    /**
      Euclidean distance between two points for 2D.
      :params p1: 2D point in space.
      :params p2: 2D point in space.
      */
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

double FindMarker::dist3D(const cv::Point3f &p1, const cv::Point3f &p2) {
    /**
      Euclidean distance between two points for 3D.
      :params p1: 3D point in space.
      :params p2: 3D point in space.
      */
    double x1 = p1.x;
    double y1 = p1.y;
    double z1 = p1.z;
    double x2 = p2.x;
    double y2 = p2.y;
    double z2 = p2.z;

    double dx = x1 - x2;
    double dy = y1 - y2;
    double dz = z1 - z2;

    return sqrt(dx*dx + dy*dy + dz*dz);
}

double FindMarker::dist3DtoCamera(const cv::Point3f &p) {
    /**
      Euclidean distance between a point for 3D in regard to camera.
      :params p: 3D point specified by its coordinates x and y and z.
      */
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

double FindMarker::calcMarkerArea(const std::vector<cv::Point2f> &pts) {
  /**
    Calculate the area of a marker in the image. Herons formula is used.
    :params pts: vector of 2D points specified by its coordinates x and y.
    */
    const cv::Point2f &p0 = pts.at(0);
    const cv::Point2f &p1 = pts.at(1);
    const cv::Point2f &p2 = pts.at(2);
    const cv::Point2f &p3 = pts.at(3);

    double a1 = dist2D(p0, p1);
    double b1 = dist2D(p0, p3);
    double c1 = dist2D(p1, p3);

    double a2 = dist2D(p1, p2);
    double b2 = dist2D(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return (a1+a2);
}


double FindMarker::calcAngleX(const cv::Point2f &p1, const cv::Point2f &p2) {
  /**
    Calculate the angle between two points.
    :params p1: 2D point in space.
    :params p2: 2D point in space.
    */
    double hyp = sqrt((p1.x - p2.x)* (p1.x - p2.x));
    double ops = sqrt((p1.y - p2.y)* (p1.y - p2.y));

    return asin(ops / hyp);
}

double FindMarker::calcAngleXAvg(std::vector<cv::Point2f> vpts1, std::vector<cv::Point2f> vpts2) {
  /**
    Calculate the angle around the z-axis using the top two markers.
    :params vpts1: vector of all the corner points.
    :params vpts2: vector of all the corner points.
    */

  double tmp = 0.0;
  // Angle between the 2 markers
  for (size_t i=0; i < 2; i++) {
    for (size_t n=0; n < 2; n++) {
      tmp += calcAngleX(vpts1[i], vpts2[n]);
    }
  }

  for (size_t i=2; i < 4; i++) {
    for (size_t n=2; n < 4; n++) {
      tmp += calcAngleX(vpts1[i], vpts2[n]);
    }
  }

  //Angles within the markers
  for (size_t i=0; i < 3; i+=2) {
    tmp += calcAngleX(vpts1[i], vpts1[i+1]);
    tmp += calcAngleX(vpts2[i], vpts2[i+1]);
  }

  return (tmp / 12);
}

cv::Mat FindMarker::GetWorldPoint(cv::Vec3d &rvecs){
  /**
    */
    cv::Mat Rmat = cv::Mat::ones(3, 3, CV_64F);
    cv::Rodrigues(rvecs, Rmat);

    return Rmat;
}

cv::Mat FindMarker::DoubleMatFromVec3b(cv::Vec3d &input){
	cv::Mat mat(3,1, CV_64F);
	mat.at <double>(0,0) = input[0];
	mat.at <double>(1,0) = input[1];
	mat.at <double>(2,0) = input[2];

	return mat;
}


