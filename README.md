# MarkerDetection

This project was done at TUM within the Chair of Building Realization and Robotics within the HEPHAESTUS project. The concept is the Computer Vision part of a construction robot. 
Goal was to conceive the translation and rotation of an object that should be grabbed by the robot properly in order to assemble a fasade. 
To estimate the pose, Aruco Markers with ids <strong>10</strong>, <strong>50</strong>, <strong>100</strong> are used. System just detects ID 10 at this stage.

FURTHER DEVELOPING STEPS:
	- using more Aruco Markers for estimating the relative position of the camera.
	- implementing the use of a second camera (done)
	- making use of the second camera in order to achieve better results.

## HEPHAESTUS project

The HEPHAESTUS project explores the innovative use of robots and autonomous systems in construction, a field where the incidence of such technologies is minor to non-existent. 
The project aims to increase market readiness and acceptance of key developments in cable robots and curtain walls.
Over the project lifetime it shall produce fundamental technical validation outside the lab, and deliver significant results such as:
	
	- a prototype cable robot, designed to build, repair and maintain a building façade;
	- a prototype building and curtain wall system, suitable for robot assembly; and
	- a business plan for widespread commercial adoption.

[a link](https://www.hephaestus-project.eu/)

## Hardware
- Camera: Basler acA 1920-40gc
- Objective: Basler Lens C10-0814-2M-S f8mm - Lens
- CPU & GPU: Nvidia Jetson TX2 Developer Kit

## Dependencies
- Ubuntu 18.04
- ROS Melodic
- OpenCV (3.2.0 x using the one from the official Ubuntu repositories is recommended)
- pylon-ROS-camera (https://github.com/basler/pylon-ros-camera)
- usb_cam (ros package)

## How to run the project
- Start the camera driver: 'roslaunch pylon_camera pylon_camera_node.launch'
- Run the detection process: 'rosrun marker_detection marker_detection'
- Ask the publisher: 'rostopic echo /coordinates'

- Run it all at once: 'roslaunch marker_detection marker_detection.launch'

## Calibration
Please consider that the camera has to be calibrated. You can use the ROS package for that.

	

