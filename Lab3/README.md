# Lab 3

## General overview

The goal of this lab is to start ROS2 client and server services.

## Useful links

1. [Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
2. [Writing a simple service and client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
3. API docs for Service Client ([old](https://docs.ros2.org/foxy/api/rclpy/api/services.html#module-rclpy.client), [new](https://docs.ros.org/en/iron/p/rclpy/rclpy.client.html)), Service Server ([old](https://docs.ros2.org/foxy/api/rclpy/api/services.html#module-rclpy.service), [new](https://docs.ros.org/en/iron/p/rclpy/rclpy.service.html))
4. [Source code for minimal service client examples](https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_client/examples_rclpy_minimal_client)
5. [Source code for minimal service server examples](https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_service/examples_rclpy_minimal_service)

## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. See Lab1 step.
2. See Lab1 step.
3. See Lab1 step.
4. See Lab1 step.
5. See Lab1 step.

6. Navigate into the `src` folder of your `ENAE450_ws` workspace and create *lab3* package:
    ```bash
    ros2 pkg create --build-type ament_python lab3_package
    ```

7. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

8. From folder `Lab3/files` copy files `client_async_member_function.py`and `service_member_function.py` into `ENAE450/ENAE450_ws/src/lab3_package/lab3_package` folder.

9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab3_package/setup.py` file and make sure *entry_points* has the following:
    ```
    'console_scripts': [
        "py_my_client = lab3_package.client_async_member_function:main",
        "py_my_server = lab3_package.service_member_function:main"
    ],
    ```

10. Navigate to `ENAE450/ENAE450_ws/src/lab3_package/lab3_package` folder and add execution permission to the python files:
    ```bash
    chmod u+x client_async_member_function.py service_member_function.py
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
    ros2 run lab3_package py_my_server
    ```
    In the other smaller pane run
    ```bash
    ros2 run lab3_package py_my_client
    ```    
    The third pane can be used to inverstigate the nodes and services.
