# Lab 4

## General overview

The goal of this lab is to create custom ROS2 message and service.

## Useful links

1. [Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. Repeat steps 1-5 from Lab1.

2. Navigate into the `src` folder of your `ENAE450_ws` workspace and create new package:
    ```bash
    ros2 pkg create --build-type ament_cmake interface_package --dependencies rclcpp
    ```

3. Create `msg` and `srv` folders in the new package (in `ENAE450/ENAE450_ws/src/interface_package`)
    ```bash
    mkdir msg srv
    ```

3. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

8. From folder `Lab4/files` copy files `SingleArray.msg`and `AddThreeInts.srv` into `ENAE450/ENAE450_ws/src/interface_package/msg` and `ENAE450/ENAE450_ws/src/interface_package/srv` folders correspondingly.

9. Edit `CMakeLists.txt` file of `interface_package` to add the following lines
    ```
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SingleArray.msg"
    "srv/AddThreeInts.srv"
    )
    ```

10. Edit `package.xml` file of `interface_package` to add the following lines
    ```
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

11. Navigate to the `ENAE450_ws` folder and build `interface_package` by running `colcon` script:
    ```bash
    colcon build --packages-select interface_package
    ```

12. Add the built package to the list of ROS2 packages available for running
    ```bash
    source install/setup.bash
    ```

14. List the available ROS2 interfaces and verify the new message and service are now available
    ```bash
    ros2 interface list
    ```
    Inverstigate structure of the new message and service.
