# Lab 6

## General overview

The goal of this lab is to demostrate launch files and to control turtlesim by a publisher.

## Useful links

1. [Using turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
2. [Creating a launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
3. [Integrating launch files into ROS 2 packages](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)

## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. Repeat steps 1-5 from Lab1.

2. Navigate into the `src` folder of your `ENAE450_ws` workspace and create new package:
    ```bash
    ros2 pkg create --build-type ament_python lab6_package --dependencies rclpy
    ```
3. Navigate into the newly created `lab6_package` package and create `launch` folder:
    ```bash
    mkdir launch
    ```

4. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

5. From folder `Lab6/files` copy the file `lab6_publisher.py` into `ENAE450/ENAE450_ws/src/lab6_package/lab6_package` folder.

6. From folder `Lab6/files` copy files `turtlesim_mimic_launch.py` and `turtlesim_mimic_control_launch.py` into `ENAE450/ENAE450_ws/src/lab6_package/launch` folder.

9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab6_package/setup.py` file and make the following changes
    * add to the imported modules:
        ```
        import os
        from glob import glob
        ```
    * modify *entry_points*:
        ```
        'console_scripts': [
            "py_lab6_pub = lab6_package.lab6_publisher:main"
        ],
        ```
    * add to the end of *data_files*:
        ```
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        ```        

9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab6_package/package.xml` file and the following:
    ```
    <exec_depend>turtlesim</exec_depend>
    <exec_depend>ros2launch</exec_depend>
    ```

10. Navigate to `ENAE450/ENAE450_ws/src/lab6_package/lab6_package` folder and add execution permission to the python files:
    ```bash
    chmod u+x lab6_publisher.py
    ```

11. Navigate to the `ENAE450_ws` folder and build the package by running `colcon` script:
    ```bash
    colcon build --symlink-install
    ```

12. Add the built package to the list of ROS2 packages available for running
    ```bash
    source install/setup.bash
    ```

13. Start *tmux* session:
    ```bash
    tmux new-session \; \split-window -h \; \select-pane -t 1 \; \split-window -v

    ```

14. In one of the smaller panes run
    ```bash
    ros2 launch lab6_package turtlesim_mimic_launch.py
    ```
    In the other panes run try controlling the turtles by manually publishing the commands. What are the differences between launch files? Try modifying parameters. Inverstigate the nodes, services, and parameters using CLI commands and rqt_graph.
