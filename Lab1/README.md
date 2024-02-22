# Lab 1

## General overview

The goal of this lab is to run a simple ROS2 code.

**Volume Map**: Docker Containers are destroyed when you exit the container, which means all the data will be lost. Volume mapping is used to save the important data before destroying the container. A directory from host pc is mapped to a directory in the container. Changes made in the mapped directory in the docker container is reflected in the host pc. Save the important data in the mapped directory inside the container.

## Steps

**The commands given in each step below are meant to be entered in the terminal unless stated otherwise**.

1. Create a folder (unless it already exists) on your PC where you can save all work in this class.
2. Open a terminal window and navigate to your folder using `cd` command.

    **For Windows PCs**
    * Open *cmd*, *Windows PowerShell*, or other terminal application of your choice
    * Start *Windows Subsystem for Linux* by typing `wsl`
    * Navigate using `cd` command

    **For Ubuntu PCs**
    * `Ctrl + Alt + T` shortcut opens terminal window
    * Navigate using `cd` command

    **For Mac Laptops**
    * Start *VMware Fusion*
    * Start Ubuntu Virtual Machine
    * Open terminal window, depending on your setup `Ctrl + Alt + T` shortcut might do it
    * Navigate using `cd` command

3. Clone the repository for this class if you haven't done it yet:

    ```bash
    git clone https://github.com/MarylandRoboticsCenter/ENAE450
    ```
    If you already cloned the repository, make sure all updates are pulled:

    ```bash
    cd ENAE450
    git pull
    ```

4. **For students who use Docker only** Periodically confirm that your Docker image is up to date. Navigate into the `ENAE450/Lab0` folder and rebuild the image:
    ```bash
    DOCKER_BUILDKIT=1 docker build --build-arg USER=$USER \
        --build-arg UID=$(id -u) \
        --build-arg GID=$(id -g) \
        --build-arg PW=docker \
        -t tb3_image \
        -f humble_dockerfile.Dockerfile\
        .
    ```

5. **For students who use Docker only** Launch Docker container that is based on the *tb3_image* image and volume map the `ENAE450_ws/src` directory in the host pc to the `ENAE450_ws/src` directory in the docker container.
    * **IMPORTANT!** Navigate into the `ENAE450/ENAE450_ws/src` folder
    * **For PCs with Nvidia GPU** Launch Docker container:
        ```bash
        docker run -it --rm --name TB3Container --net=host --ipc=host --pid=host --gpus=all --runtime=nvidia --privileged \
            --env="DISPLAY=$DISPLAY" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="$PWD:/home/${USER}/ENAE450_ws/src" \
            tb3_image:latest
        ```
    * **For PCs without GPU acceleration** Launch Docker container:
        ```bash
        docker run -it --rm --name TB3Container --net=host --ipc=host --pid=host --privileged \
            --env="DISPLAY=$DISPLAY" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="$PWD:/home/${USER}/ENAE450_ws/src" \
            tb3_image:latest
        ```
    **Note** The terminal is now in the Docker container. Keep in mind that all changes and installations will reset when you exit the container. However, the changes made in the `~/ENAE450_ws/src` folder will stay since it is volume mapped to your host PC folder `ENAE450/ENAE450_ws/src`.

**Note** The above steps will be common for all of the labs and I will refer to them in the beginning of each lab guide. 

6. Navigate into the `src` folder of your `ENAE450_ws` workspace and create *lab1* package:
    ```bash
    ros2 pkg create --build-type ament_python lab1_package
    ```

7. Open VS Code (or other IDE of choice) on your host PC and open `ENAE450_ws` folder

8. From folder `Lab1/files` copy files `simple_node1.py`, `simple_node2.py`, and `node_class.py` into `ENAE450/ENAE450_ws/src/lab1_package/lab1_package` folder.

9. **In VS Code** Edit `ENAE450/ENAE450_ws/src/lab1_package/setup.py` file and make sure *entry_points* has the following:
    ```
    'console_scripts': [
        "py_simple_node1 = lab1_package.simple_node1:main",
        "py_simple_node2 = lab1_package.simple_node2:main",
        "py_node_class = lab1_package.node_class:main"
    ],
    ```        

10. Navigate to `ENAE450/ENAE450_ws/src/lab1_package/lab1_package` folder and add execution permission to the python files:
    ```bash
    chmod u+x simple_node1.py simple_node2.py node_class.py 
    ```

11. Navigate to the `ENAE450_ws` folder and build the package by running `colcon` script:
    ```bash
    colcon build --symlink-install
    ```

12. Add the built package to the list of ROS2 packages available for running
    ```bash
    source install/setup.bash
    ```

13. Try running nodes:
    ```bash
    ros2 run lab1_package py_simple_node1
    ```


