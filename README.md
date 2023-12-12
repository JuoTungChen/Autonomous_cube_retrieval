# Autonomous Cube Retrieval

## Description
This project addresses the challenge of autonomously navigating a robotic arm, specifically a UR5 manipulator, to pick up a cube in an environment where the initial positions of the robot's base frame, the RealSense camera, and the cube are unknown. The objective is to develop an integrated solution that combines computer vision techniques, registration, and robotic control strategies to achieve precise and reliable manipulation.
![setup](https://github.com/JuoTungChen/Autonomous_cube_retrieval/raw/master/images/setup.jpg)

## Table of Contents
- [Approach Overview](#approach-overview)
- [Installation and Usage](#installation-and-usage)
- [Improvements and Future Work](#improvements-and-future-work)

## Approach Overview
1. Object Detection and Segmentation

    Utilizing a classical computer vision approach (color thresholding, Canny edge detection, and corner detection), the system is able to identify and segment the cube.
    ![Cube Corner](https://github.com/JuoTungChen/Autonomous_cube_retrieval/raw/master/images/cube_corner.png)

2. Pose Estimation

    The [Perspective-n-Point (PnP) algorithm](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) was employed to determine the pose of the cube relative to the RealSense camera. By utilizing the 2D image coordinates of cube corners in the camera frame and their corresponding 3D coordinates in the world frame, PnP efficiently calculated the rotation and translation vectors, providing a precise spatial understanding of the cube's position and orientation in the camera's field of view. This information was crucial for subsequent robotic manipulation tasks.
    ![Pnp](https://github.com/JuoTungChen/Autonomous_cube_retrieval/raw/master/images/solve_pnp.png)

    ![pose](https://github.com/JuoTungChen/Autonomous_cube_retrieval/raw/master/images/pose_estimation.png)


3. Registration Between Camera and Robot

    The [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar?tab=readme-ov-file) package was employed for camera-to-robot registration by detecting a predefined anchor marker in the RealSense camera's field of view. This marker had a known relative pose with respect to the base_link of the UR5 robot. Using the detected marker's pose, we can establish a transformation between the camera and the robot base.

4. Robot Control Using MoveIt!

    Finally, using Moveit for motion planning, the robot can follow a collision-free motion plan for the pick and place task. By instructing the robot to align its end-effector with the estimated pose of the cube, the robot can subsequently close the gripper and pick the cube up.

## Installation and Usage

### Prerequisite
The demo was run on __Ubuntu 18.04 LTS__, with __ROS Melodic__ installed.

### Step-by-Step Guide

First, make sure you already have installed the libraries and ROS wrapper for the realsense camera: [tutorial](https://github.com/leggedrobotics/realsense-ros-rsl)

Next, launch the realsense camera
```
roslaunch realsense2_camera rs_camera.launch depth_align:=true  
```
The rgb image topic should be `/camera/color/image_raw`, and the camera info topic should be `/camera/color/camera_info`

Next, run the `cube_pose_estimation.py` file for cube segmentation and pose estimation.

In order to launch the robot, you have to first create a workspace:
```
mkdir -p ur5_ws/src
cd ur5_ws/src
```

git clone this [repo](https://github.com/intuitivecomputing/ur5_with_robotiq_gripper)

```
git clone https://github.com/intuitivecomputing/ur5_with_robotiq_gripper
```

Then, build and source the path:

```
cd ../
catkin build
source ./devel/setup.bash
```


launch the following in seperate terminals in order to control the robot (remember to source the path in each terminal):

```
roslaunch icl_ur5_setup_bringup ur5_gripper.launch

roslaunch icl_ur5_setup_bringup activate_gripper.launch

roslaunch icl_ur5_setup_moveit_config ur5_gripper_moveit_planning_execution.launch

roslaunch icl_ur5_setup_moveit_config moveit_rviz.launch config:=true

```

After that, make sure the ar_track_alvar package is installed:

```
cd ~/ur5_ws/src
git clone https://github.com/ros-perception/ar_track_alvar?tab=readme-ov-file
```

put the `tag_detector_rs.launch` into the `ar_track_alvar/launch` folder and build the workspace.

After making sure the realsense can see the ar marker, run the launch file:
```
roslaunch ar_track_alvar tag_detector_rs.launch
```

Now you should be able to see the camera frame, the ar marker, as well as the pose of the cube in rviz like this:
![rviz](https://github.com/JuoTungChen/Autonomous_cube_retrieval/raw/master/images/rviz_frames.jpg)

Finally, run `cube_pick_and_place.py` and the robot will pick the cube up and then drop it down!

[Demo Video](https://livejohnshopkins-my.sharepoint.com/:v:/g/personal/jchen396_jh_edu/EaxSxu72_slAuGWwJnBTQM0BbN5iQUr2krsX-Tbl1lTOKQ?e=qGxtSd&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifX0%3D) 

## Improvements and Future Work
With limited time, this demo was just to demonstrate the possiblity of using the UR5 robot for autonomous cube retrieval task. In order to make the system more robust, the following future work should be implemented and explored:

1. Despite realsense camera already have on-chip calibration,  proper [camera calibration procedure](https://wiki.ros.org/camera_calibration) can still be conducted to improve the pose estimation accuracy and rectify the image distortion.

2. Right now, to detect and estimate the pose of the cube, classical CV techniques were employed. However, there is still limitations for classical approach. The first major issue is that color thresholding will limit the color of the object and lighting can also have an influence on the detection accuracy. To streamline the process and improve the accuracy, [PoseCNN and Decoder](https://docs.nvidia.com/isaac/archive/2021.1/packages/object_pose_estimation/doc/pose_cnn_decoder.html) can be employed and train on the specific object.

3. Also, to improve the pose estimation accuracy, instead of using the Perspective-n-point algorithm, the [Iterative Closest Point](https://www.sciencedirect.com/topics/engineering/iterative-closest-point-algorithm#:~:text=The%20goal%20of%20the%20ICP,Systems%20for%20Smart%20Trains%2C%202021) algorithm can also be used in order to utilize the point cloud generated from the Realsense camera. Here is a in-depth guide on [how to use ICP for pose estimation](https://manipulation.csail.mit.edu/pose.html)

4. Finally, to have a more robust registration between the camera and the robot base, instead of using an anchor AR marker, a calibration procedure can be taken. This approach is to first use the gripper to grasp an AR marker, and then move the robot to various points within the camera view. For each pose, we can establish the following relationship: 
    - robot base to end-effector (gripper center): can be calculated using forward kinematics
    - camera to AR marker: can be detected using packages like ar_track_alvar

    If we can make sure the AR marker is aligned with the gripper center, then the transformation between camera to the robot base can be established.

    This approach has already been implemented in my another project using ROS2: [AR pick and place](https://github.com/vishnukolal/rsp_project)


