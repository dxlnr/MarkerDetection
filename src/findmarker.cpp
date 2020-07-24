/**
  * listener.cpp
  *
  *  Created by: Daniel Illner on 10.05.2020
  */

#include "findmarker.h"
#include "poseEstimation.h"


FindMarker::FindMarker(cv::Mat image){
  _image = image;
}

FindMarker::~FindMarker(){
}

void FindMarker::detectContours(){

  cv::cvtColor(_image, _image, cv::COLOR_BGR2RGB);
  //std::cout << "Image channels after BGR2RGB: " << _image.channels() << std::endl;
  //std::cout << "image = " << std::endl << " " << _image << std::endl << std::endl;

  /// Converting to grayscale
  cv::cvtColor(_image, grayScale, cv::COLOR_BGR2GRAY);

  ///Thresholding and converting to binary
  cv::threshold(grayScale, grayScale, threshold_value, 255, cv::THRESH_BINARY);

  cv::Mat test = grayScale;
  //std::cout << "Test = " << std::endl << " "  << grayScale << std::endl << std::endl;

  /// Finding contours in the image.
  contour_vector_typ contours;
  cv::findContours(grayScale, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  bool isFirstStripe = true;
  bool isFirstMarker = true;

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

    if (r.height < 20 || r.width < 20 || r.width > _image.cols - 10 || r.height > _image.rows - 10) {
      continue;
    }

    //cv::polylines(_image, approx_contour, true, colour, thickness_value);

    //float lineParams[16];
    cv::Mat lineParamsMat(cv::Size(4, 4), CV_32F, lineParams);

    // --- Process Corners ---
    for (size_t i = 0; i < approx_contour.size(); i++) {
      // Iterate through every line of every rectangle and try to find the corner points.
      //cv::circle(_image, approx_contour[i], 3, CV_RGB(255, 255, 0), -1);

      double dx = ((double)approx_contour[(i + 1) % 4].x - (double)approx_contour[i].x) / 7.0;
      double dy = ((double)approx_contour[(i + 1) % 4].y - (double)approx_contour[i].y) / 7.0;

      cv::Mat iplStripe = calculate_Stripe(dx, dy, stripe);

      cv::Point2f points[6];

      for (size_t j = 0; j < 7; j++){

        // Position calculation
        double px = (double)approx_contour[i].x + (double)j * dx;
        double py = (double)approx_contour[i].y + (double)j * dy;

        cv::Point p;
        p.x = (int)px;
        p.y = (int)py;
        //cv::circle(_image, p, 2, CV_RGB(0, 255, 255), -1);

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


            cv::Point p2;
            p2.x = (int)subPixel.x;
            p2.y = (int)subPixel.y;


            //std::cout << points[j] << std::endl;


            /*
            if (isFirstStripe)
              circle(_image, p2, 1, CV_RGB(255, 0, 255), -1);
            else
              circle(_image, p2, 1, CV_RGB(0, 255, 255), -1);
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
        //cv::circle(_image, edgeCenter, 2, CV_RGB(0, 200, 200), -1);

        points[j - 1].x = edgeCenter.x;
        points[j - 1].y = edgeCenter.y;
      }


      // lineParams is shared
      cv::Mat point_mat(cv::Size(1, 6), CV_32FC2, points);
      cv::fitLine (point_mat, lineParamsMat.col(i), cv::DIST_L2, 0, 0.01, 0.01 );

      // Two points needed to draw the line.
      //cv::Point pt1;
			pt1.x = (int)lineParams[8 + i] - (int)(50.0 * lineParams[i]);
			pt1.y = (int)lineParams[12 + i] - (int)(50.0 * lineParams[4 + i]);

			//cv::Point pt2;
			pt2.x = (int)lineParams[8 + i] + (int)(50.0 * lineParams[i]);
			pt2.y = (int)lineParams[12 + i] + (int)(50.0 * lineParams[4 + i]);

      cv::line(_image, pt1, pt2, CV_RGB(0, 255, 255), 1, 8, 0);

    }

    // Compute the intersection between two line segments & save the corner points.
    computeIntersection();

    //to get the matrix of perspective transform
    setReferenzPoints();
    cv::Mat projMat(cv::Size(3, 3), CV_32FC1);
    projMat = cv::getPerspectiveTransform(corners, set2Points);

    //create Marker Image
    cv::Mat imageMarker(cv::Size(6, 6), CV_8UC1);
    cv::warpPerspective(grayScale, imageMarker, projMat, cv::Size(6,6));

    //threshold the marker image to get a B/W image
    //cv::threshold(...)

    // ### Identify the marker ###

    int code = 0;
		for (int i = 0; i < 6; ++i) {
			// Check if border is black
			int pixel1 = imageMarker.at<uchar>(0, i); //top
			int pixel2 = imageMarker.at<uchar>(5, i); //bottom
			int pixel3 = imageMarker.at<uchar>(i, 0); //left
			int pixel4 = imageMarker.at<uchar>(i, 5); //right

			// 0 -> black
			if ((pixel1 > 0) || (pixel2 > 0) || (pixel3 > 0) || (pixel4 > 0)) {
				code = -1;
        break;
			}
		}

		if (code < 0) {
      continue;
		}

    //std::cout << "Found a marker." << std::endl;

    int cP[4][4];
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				// +1 -> no borders!
				cP[i][j] = imageMarker.at<uchar>(i + 1, j + 1);
				// If black then 1 else 0
				cP[i][j] = (cP[i][j] == 0) ? 1 : 0;
			}
		}

		// Save the ID of the marker, for each side
		int codes[4];
		codes[0] = codes[1] = codes[2] = codes[3] = 0;

    // Calculate the code from all sides at once
		for (int i = 0; i < 16; i++) {
			// /4 to go through the rows
			int row = i >> 2;
      int col = i % 4;

			// Multiplied by 2 to check for black values -> 0*2 = 0
			codes[0] <<= 1;
			codes[0] |= cP[row][col]; // 0\B0

			// 4x4 structure -> Each column represents one side
			codes[1] <<= 1;
			codes[1] |= cP[3 - col][row]; // 90\B0

			codes[2] <<= 1;
			codes[2] |= cP[3 - row][3 - col]; // 180\B0

			codes[3] <<= 1;
			codes[3] |= cP[col][3 - row]; // 270\B0
		}

    // Account for symmetry -> One side complete white or black
		if ((codes[0] == 0) || (codes[0] == 0xffff)) {
      continue;
		}

    int angle = 0;

		// Search for the smallest marker ID
		code = codes[0];
		for (int i = 1; i < 4; ++i) {
			if (codes[i] < code) {
				code = codes[i];
        angle = i;
			}
		}
    std::cout << codes[0] << " " << codes[1] << " " << codes[2] << " " << codes[3] << std::endl;

		// Print ID
		printf("Found: %04x\n", code);
    printf("Angle: %i\n", angle);


    if (angle != 0) {
      cv::Point2f corrected_corners[4];
			// Smallest id represents the x-axis, we put the values in the corrected_corners array
			for (int i = 0; i < 4; i++) {
        corrected_corners[(i + angle) % 4] = corners[i];
      }

			// We put the values back in the array in the sorted order
			for (int i = 0; i < 4; i++)	{
        corners[i] = corrected_corners[i];
      }
		}


		for (int i = 0; i < 4; i++) {
			corners[i].x -= camera_resolution_x * 0.5;
			// -(corners.y) -> is neeeded because y is inverted
			corners[i].y = -corners[i].y + (camera_resolution_y * 0.5);
		}

		// 4x4 -> Rotation | Translation
		//        0  0  0  | 1 -> (Homogene coordinates to combine rotation, translation and scaling)
		float resultMatrix[16];
		// Marker size in meters!
		estimateSquarePose(resultMatrix, (cv::Point2f*)corners, 0.04346);

		// This part is only for printing
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				std::cout << std::setw(6); // Total 6
				std::cout << std::setprecision(4); // Numbers of decimal places = 4 (of the 6)

				std::cout << resultMatrix[4 * i + j] << " ";
			}
			std::cout << "\n";
		}
		std::cout << "\n";
		float x, y, z;
		// Translation values in the transformation matrix to calculate the distance between the marker and the camera
		x = resultMatrix[3];
		y = resultMatrix[7];
    z = resultMatrix[11];

		// Euclidian distance
		std::cout << "length: " << sqrt(x* x + y * y + z * z) << "\n";
	  std::cout << "\n";



  }

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  cv::resizeWindow("view", 2560, 1980);
  cv::imshow("view", _image);
  //cv::Mat testtest;
  //cv::cvtColor(grayScale, testtest, CV_GRAY2RGB);
  //cv::imshow("view", testtest);
  cv::waitKey(30);

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

/*
void FindMarker::fittingLine(){

}
*/

void FindMarker::computeIntersection() {
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
      std::cout << "lines parallel" << std::endl;
      continue;
    }

    // We have checked for parallelism of the direction vectors
    // -> Cramer's rule, now divide through the main determinant
    a /= c;
    b /= c;

    // Exact corner
    corners[i].x = a;
    corners[i].y = b;

    //cv::Point pt;
    pt.x = (int)corners[i].x;
    pt.y = (int)corners[i].y;

    cv::circle(_image, pt, 5, CV_RGB(0, 0, 255), -1);
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

/*
void FindMarker::identifyMarker(){

}
*/
