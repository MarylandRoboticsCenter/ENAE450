# Extra: video streams in ROS and tracking fiducial (ArUco) markers

## General overview

Demonstration of ArUco markers tracking capabilities in ROS2

## Useful links

1. [OpenCV ArUco marker detection library ](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
2. [ROS2 package for ArUco markers (one of many)](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco)
3. [Description of image_transport package in ROS1](https://wiki.ros.org/image_transport)
4. [Description of image_proc package in ROS1](https://wiki.ros.org/image_proc)
5. [rosdep tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)


## Main (only) Steps

1. Clone to your PC workspace `ros2_aruco` package (`opencv_4.7` branch)
    ```bash
    git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco -b opencv_4.7
    ```

2. Copy the included launch files to the `launch` folder

3. Copy the included config file to the `ros2_aruco/config` folder

4. Install the workspace dependencies
    ```bash
    pip3 install opencv-contrib-python transforms3d
    rosdep install --from-paths src -y --ignore-src
    ```
5. Build and source the workspace

6. Bringup TB3 by launching `robot_picam_rplidar.launch.py`

7. Launch (on your PC) `image_auxil.launch.py`

8. Launch (on your PC) `image_auxil.launch.py`

