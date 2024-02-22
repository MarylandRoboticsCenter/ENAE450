# Lab 2

## General overview

The goal of this lab is start ROS2 publisher and subscriber nodes.

## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. See Lab1 step.
2. See Lab1 step.
3. See Lab1 step.
4. See Lab1 step.
5. See Lab1 step.

6. Navigate into the `src` folder of your `ENAE450_ws` workspace and create *lab2* package:
    ```bash
    ros2 pkg create --build-type ament_python lab2_package
    ```

7. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

8. From folder `Lab2/files` copy files `my_publisher.py`and `my_subscriber.py` into `ENAE450/ENAE450_ws/src/lab2_package/lab2_package` folder.

9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab2_package/setup.py` file and make sure *entry_points* has the following:
    ```
    'console_scripts': [
        "py_my_publisher = lab2_package.my_publisher:main",
        "py_my_subscriber = lab2_package.my_subscriber:main"
    ],
    ```

10. Navigate to `ENAE450/ENAE450_ws/src/lab2_package/lab2_package` folder and add execution permission to the python files:
    ```bash
    chmod u+x my_publisher.py my_subscriber.py
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
    ros2 run lab2_package py_my_publisher
    ```
    In the other smaller pane run
    ```bash
    ros2 run lab2_package py_my_subscriber
    ```    
    The third pane can be used to inverstigate the nodes and topics connections.
