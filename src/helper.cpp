/**
  * helper.cpp
  *
  *  Created by: Daniel Illner on 10.04.2020
  */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <experimental/filesystem>
#include <boost/filesystem.hpp>

#include "helper.h"

helper::helper(){

}

helper::~helper(){

}

std::string helper::resolvePath(const std::string &relPath)
{
    namespace fs = boost::filesystem;
    auto baseDir = fs::current_path();

    //std::cout << "relative path: " << relPath << std::endl;
    //std::cout << "parentdir: " << baseDir << std::endl;

    while (baseDir.has_parent_path())
    {
        auto combinePath = baseDir / relPath;
        if (fs::exists(combinePath))
        {
            return combinePath.string();
        }
        baseDir = baseDir.parent_path();
    }
    throw std::runtime_error("File not found!");
}
