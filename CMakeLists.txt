cmake_minimum_required(VERSION 2.8.3)

### ROS ###

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

#ADD_DEFINITIONS(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  	     roscpp
  	     rospy
  	     std_msgs
	     tf
  	     geometry_msgs
	     sensor_msgs
	     message_generation
	     message_filters
  	     cv_bridge
  	     image_transport
  	     cmake_modules
)

project(marker_detection)

###################################
## Generate messages  		 ##
###################################

add_message_files(
   FILES
   reference.msg
   MarkerVertices.msg
 )

generate_messages(
   DEPENDENCIES
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES marker_detection
	CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs message_runtime message_filters cv_bridge image_transport
	DEPENDS system_lib
        DEPENDS message_runtime std_msgs
)


####################
## Building Block ##
####################

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

### PCL ###

find_package(PCL 1.8.1 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


### OpenCV ###

set(OpenCV_DIR $ENV{OpenCV_DIR})
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_aruco opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect)




### Sources to compile ###

add_executable(marker_detection src/poseEstimation.cpp src/main.cpp src/helper.cpp src/findmarker.cpp)
target_link_libraries(marker_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(marker_detection ${catkin_EXPORTED_TARGETS})



######################
## Installing Block ##
######################

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############
