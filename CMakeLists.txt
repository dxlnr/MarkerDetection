cmake_minimum_required(VERSION 2.8.3)

### ROS ### 

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

#ADD_DEFINITIONS(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  	     roscpp
  	     rospy
  	     std_msgs
	     tf
  	     sensor_msgs
  	     nav_msgs
  	     visualization_msgs
  	     cv_bridge
  	     image_transport
  	     cmake_modules
  	     OpenCV REQUIRED
)

project(marker_detection)


###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES marker_detection
	CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge image_transport
	DEPENDS system_lib
        DEPENDS message_runtime std_msgs 
)


####################
## Building Block ##
####################

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


### OpenCV ###

set(OpenCV_DIR $ENV{OpenCV_DIR})
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


### Sources to compile ###

add_executable(marker_detection src/listener.cpp src/main.cpp src/helper.cpp)
target_link_libraries(marker_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(marker_detection ${catkin_EXPORTED_TARGETS})



######################
## Installing Block ##
######################


#############
## Testing ##
#############


