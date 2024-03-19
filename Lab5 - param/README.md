# Lab 5

## General overview

The goal of this lab is to demonstrate implementation of ROS2 parameters, custom messages and services.

## Useful links

1. [Understanding parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
2. [Using parameters in a class (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
3. API docs for Parameters ([old](https://docs.ros2.org/foxy/api/rclpy/api/parameters.html), [new](https://docs.ros.org/en/iron/p/rclpy/rclpy.parameter.html))

## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. Repeat steps 1-5 from Lab1.

2. Navigate into the `src` folder of your `ENAE450_ws` workspace and create new package:
    ```bash
    ros2 pkg create --build-type ament_python lab5_package --dependencies rclpy
    ```

3. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

8. From folder `Lab5/files` copy files `lab5_publisher.py`and `lab5_subscriber.py` into `ENAE450/ENAE450_ws/src/lab5_package/lab5_package` folder.

9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab5_package/setup.py` file and make sure *entry_points* has the following:
    ```
    'console_scripts': [
        "py_lab5_sub = lab5_package.lab5_subscriber:main",
        "py_lab5_pub = lab5_package.lab5_publisher:main"
    ],
    ```
9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab5_package/package.xml` file and the following:
    ```
    <build_depend>interface_package</build_depend>
    <exec_depend>interface_package</exec_depend>
    ```

10. Navigate to `ENAE450/ENAE450_ws/src/lab5_package/lab5_package` folder and add execution permission to the python files:
    ```bash
    chmod u+x lab5_subscriber.py lab5_publisher.py
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
    ros2 run lab5_package py_lab5_pub
    ```
    In the other smaller pane run
    ```bash
    ros2 run lab5_package py_lab5_sub
    ```    
    The third pane can be used to inverstigate the nodes and services.
