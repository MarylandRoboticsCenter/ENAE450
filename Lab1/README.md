# Lab 1

## General overview

The goal of this lab is to run a simple ROS2 code.

**Volume Map**: Docker Containers are destroyed when you exit the container, which means all the data will be lost. Volume mapping is used to save the important data before destroying the container. A directory from host pc is mapped to a directory in the container. Changes made in the mapped directory in the docker container is reflected in the host pc. Save the important data in the mapped directory inside the container.

## Steps

**The commands given in each step below are meant to be copied and pasted in the terminal**.

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
    * Start Ubuntu Virtual opens terminal window
    * Open terminal window, depending on your setup `Ctrl + Alt + T` shortcut might do it
    * Navigate using `cd` command

3. Clone the repository for this lab:

    ```bash
    git clone https://github.com/MarylandRoboticsCenter/ENAE450/tree/main/Lab1
    ```

2. Create a workspace folder, e.g. `ENAE450_ws` in your filesystem (not in docker container)
2. Create `src` folder in the workspace folder
3. Navigate into the `src` folder
4. **This step is not needed for people who don't run Docker.** Now, you will create a docker container based on the `tb3_image` image which you built earlier and volume map the `src` directory in the host pc to the `src` directory in the docker container. To do that, enter the version of the following command that works for you (make sure that you are in the `../src`` directory inside the terminal before running this command):
    ```bash
    docker run -it --rm --name TB3Container --net=host --ipc=host --pid=host --gpus=all --runtime=nvidia --privileged \
        --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$PWD:/home/${USER}/catkin_ws/src" \
        tb3_image:latest
    ```
