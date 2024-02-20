# Software installation

* Download [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) image
* Install [Docker Desktop](https://docs.docker.com/desktop/install/linux-install/) on Linux
* Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on Ubuntu
* Install [VSCode](https://code.visualstudio.com/docs/setup/linux) on Linux
* Install [VSCode](https://code.visualstudio.com/download) on Windows

# Software help

* [ROS2 Humble](https://docs.ros.org/en/humble/Tutorials.html)
* [ROS2 API](https://docs.ros.org/en/humble/API-Docs.html)
* [Git](https://git-scm.com/docs/user-manual)
* Linux [terminal](https://linuxcommand.org/lc3_learning_the_shell.php), [cheat sheet](https://cheatography.com/davechild/cheat-sheets/linux-command-line/)
* Linux file manager ([Midnight Commander](https://linuxcommand.org/lc3_adv_mc.php)
)
* Tmux [cheat sheet](https://tmuxcheatsheet.com/)

# ROS2 concepts

* Nodes [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
* Topics [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
* Services [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
* Parameters [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
* Actions [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

# Commonly used commands

1. Terminal

    * `sudo apt-get update`
    * `sudo apt-get upgrade`
    * `cd` 
    * `mkdir`
    * `ls -al`
    * `mc`
    * `env | grep ROS`

2. Docker
    
    * docker [build](https://docs.docker.com/engine/reference/commandline/image_build/)
    ```bash
    DOCKER_BUILDKIT=1 docker build --build-arg USER=$USER \
        --build-arg UID=$(id -u) \
        --build-arg GID=$(id -g) \
        --build-arg PW=docker \
        -t tb3_image \
        -f humble_dockerfile.Dockerfile\
        .
    ```
    * docker [run](https://docs.docker.com/engine/reference/commandline/container_run/)
    ```bash
    docker run -it --rm --name TB3Container --net=host --ipc=host --pid=host --gpus=all --runtime=nvidia --privileged \
        --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$PWD:/home/${USER}/catkin_ws/src" \
        tb3_image:latest
    ```    
3. Git

    * `git clone`
    * `git fetch`
    * `git status`
    * `git pull`
    * `git stash`

3. ROS2 
  
    * `ros2 pkg create --build-type ament_python <package-name>`
    * `ros2 pkg list`
    * `colcon build --symlink-install`
    * `source ./install/setup.bash`
    * `ros2 node list`
    * `ros2 topic list`
    * `ros2 topic echo <topic name>`



