# Lab 7

## General overview

The goal of this lab is to introduce Gazebo robot simulator, describe URDF format, and to get to know Turtlebot3 simulation.

## Useful links

1. [Concept of transforms (TF) in ROS](https://articulatedrobotics.xyz/ready-for-ros-6-tf/)
2. [Summary of TF in ROS2](https://nu-msr.github.io/ros_notes/ros2/tf.html)
3. [Summary of URDF in ROS2](https://nu-msr.github.io/ros_notes/ros2/modeling.html)
4. [URDF tutorial: building robot model from scratch](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)


## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. Repeat steps 1-5 from Lab1.

2. Navigate into the `src` folder of your `ENAE450_ws` workspace and create new package:
    ```bash
    ros2 pkg create --build-type ament_python lab7_package --dependencies rclpy
    ```

3. Navigate into the newly created `lab7_package` package and create `launch` folder:
    ```bash
    mkdir launch
    ```

4. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

5. From folder `Lab7/files` copy the file `urdf_test.launch.py` into `ENAE450/ENAE450_ws/src/lab7_package/launch` folder.

6. **In VS Code** Edit the `ENAE450/ENAE450_ws/src/lab7_package/package.xml` file and add the following
    ```
    <exec_depend>sam_bot_description</exec_depend>
    ```

7. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab7_package/setup.py` file and make the following changes
    * add to the imported modules:
        ```
        import os
        from glob import glob
        ```
    * add to the end of *data_files*:
        ```
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        ```        

8. Pull and build `nav2_tutorials` repo. If you are using Docker, it should be already built as long as you have the latest version of the `Dockerfile`. If you don't use Docker, run the following
    ```bash
    mkdir -p $HOME/ENAE450_ws/src
    cd $HOME/nav_ws/src
    git clone https://github.com/ros-planning/navigation2_tutorials
    cd $HOME/nav_ws
    colcon build --symlink-install --packages-select sam_bot_description
    echo 'source $HOME/nav_ws/install/setup.bash' >> $HOME/.bashrc
    ```

9. Navigate to the `ENAE450_ws` folder and build the package by running `colcon` script:
    ```bash
    colcon build --symlink-install
    ```

10. Add the built package to the list of ROS2 packages available for running
    ```bash
    source install/setup.bash
    ```

11. Start *tmux* session:
    ```bash
    tmux new-session \; \split-window -h \; \select-pane -t 1 \; \split-window -v

    ```

14. In one of the smaller panes run
    ```bash
    ros2 launch lab7_package urdf_test.launch.py
    ```
    In the other panes run try controlling the robot by manually publishing the commands. Familiarize yourself with RViz. Inverstigate the following topics in RViz and in the terminal: 
    * /tf
    * /tf_static
    * /demo/imu
    * /scan
    * /depth_camera/image_raw

    Build the tf tree by running
    ```bash
    ros2 run tf2_tools view_frames
    ```
    Investigate the tf tree and see if you can find similarities with the urdf file (located in 'nav_ws/src/navigation2_tutorials/sam_bot_description/src/description/sam_bot_description.urdf')

15. Stop the launch file execution

16. Launch TurtleBot3 [Gazebo simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

     ```bash
    ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```
    Repeat investigation steps for the Turtlebot3 robot. What are the differences in this simulation?
