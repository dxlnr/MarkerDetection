# MarkerDetection

	This project was done at TUM within the Chair of Building Realization and Robotics. The concept is the vision part of a construction robot. Goal was to conceive the translation and rotation of an object that should be grabbed by the robot properly in order to assemble a fasade. 
	To estimate the pose, Aruco Markers with ids <strong>10</strong>, <strong>50</strong>, <strong>100</strong> are used.

## Hardware
	- Camera: Basler acA 1920-40gc
	- Objective: Basler Lens C10-0814-2M-S f8mm - Lens
	- CPU & GPU: Nvidia Jetson TX2 Developer Kit

## Dependencies
	- Ubuntu 18.04
	- ROS Melodic
	- OpenCV (4.2.0 x using the one from the official Ubuntu repositories is recommended)
	- pylon-ROS-camera (https://github.com/basler/pylon-ros-camera)

## How to run the project
	- Start the camera driver: 'roslaunch pylon_camera pylon_camera_node.launch'
	- Run the detection process: 'rosrun marker_detection marker_detection'
	- Ask the publisher: 'rostopic echo /coordinates'

## Calibration
	Please consider that the camera has to be calibrated. You can use the ROS package for that.

	

