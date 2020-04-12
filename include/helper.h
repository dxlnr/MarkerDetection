/**
  * helper.h
  *
  *  Created by: Daniel Illner on 10.04.2020
  */


#include <ros/ros.h>
#include <tf/transform_listener.h>


#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#ifndef _HELPER_H_
#define _HELPER_H_


class helper{

  public:

    helper();
    ~helper();
    std::string resolvePath(const std::string &relPath);

  private:


};
#endif
