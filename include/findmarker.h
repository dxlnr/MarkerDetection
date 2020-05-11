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




class FindMarker {

  public:
    FindMarker();
    ~FindMarker();

  private:

    // List of points
    typedef std::vector<cv::Point> contour_typ;
    // List of contours
    typedef std::vector<contour_typ> contour_vector_typ;



};

#endif
