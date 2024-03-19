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
* Linux file manager ([Midnight Commander](https://linuxcommand.org/lc3_adv_mc.php))
* Tmux [cheat sheet](https://tmuxcheatsheet.com/)
* [colcon](https://colcon.readthedocs.io/en/released/)
* [Gazebo](https://gazebosim.org/docs)

# ROS2 books

* A Gentle Introduction to ROS by Jason M. O’Kane, [link](https://cse.sc.edu/~jokane/agitr/)
* ROS Robot Programming, [link](https://www.robotis.us/ros-robot-programming-book-digital-copy/)

# ROS2 concepts

* Nodes [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
* Topics [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
* [Interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
* Services [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
* Parameters [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
* Actions [1](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html), [2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
* Launch files [1](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html), [2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
* tf2 [1](https://wiki.ros.org/tf2), [2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
* [URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
* [RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)

# ROS2 workspace structure
```
<workspace_name> (e.g. ENAE450_ws)
├── build                   # created by colcon
├── install                 # created by colcon
├── log                     # created by colcon
├── src                     # created by user
    ├── package_#1
    ├── package_#2
    ├── ...
    └── package_#N
```
# ROS2 package structure 

## Python
from https://nu-msr.github.io/ros_notes/ros2/colcon.html
```
<pkg_name>/
├── package.xml             # package manifest
├── <pkg_name>/             # python package
│   ├── __init__.py         # Marks this directory as a python package
│   └── module.py           # A python module
├── launch/                 # Launchfiles go here
│   ├── file.launch.xml     # An xml launch file
│   ├── file.launch.py      # A python launch file
│   └── file.launch.yaml    # A yaml launch file
├── config/                 # Configuration directory
│   ├── parameters.yaml     # File storing parameters for some nodes
│   └── view.rviz           # Saved rviz configuration
├── resource/               # Used for registering packages in ament_index
│   └── package_name        # Empty file used to register package with index
├── setup.cfg               # Sets up installation directories for ROS
├── setup.py                # Metadata, node entry , other files to install
└── test/                   # Unit tests
    ├── test_copyright.py   # ROS test to ensure proper copyright notice 
    ├── test_flake8.py      # Uses flake8 to enforce code style 
    └── test_pep257.py      # Ensures compliance with PEP 257
```

# ROS2 workflow

1. New package
    * create workspace (if not created)
    * create package
    * edit package metadata
    * write code (divide into nodes)
    * build package
    * source package
    * start node(s)

2. Existing package
    * edit code
    * edit package metadata if needed
    * build package
    * source package
    * start node(s)

# Commonly used commands

1. Terminal

    * `<Tab> <Tab>` - **autocomplete**
    * `sudo apt-get update`
    * `sudo apt-get upgrade`
    * `cd <dir_name>` 
    * `mkdir <dir_name>`
    * `ls -al`
    * `mc`
    * `env | grep ROS`
    * `cat <file_name>`
    * `touch <file_name>`
    * `chmod u+x <file_name>`

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
  
    * `<Tab> <Tab>` - **autocomplete**
    * `source ./install/setup.bash`
    * `colcon build --symlink-install`
    <br/><br/>
    * `ros2 pkg create --build-type ament_python <package_name>`
    * `ros2 pkg list`
    <br/><br/>
    * `ros2 node list`
    * `ros2 node info <node_name>`
    <br/><br/>
    * `ros2 topic list`
    * `ros2 topic type <topic_name>`
    * `ros2 topic echo <topic_name>`
    * `ros2 interface show <type_name>`
    * `ros2 service list`
    * `ros2 service type <service_name>`
    * `ros2 service call <service_name> <service_type> <arguments>`
    <br/><br/>
    * `ros2 run <package_name> <executable_name>`
    

# Shortcuts

1. Terminal

    * Ctrl-d - close terminal
    * Ctrl-c - abort running program
    * Ctrl-r - reverse history search

2. Tmux

    * Ctrl-d - close window/pane
    * Ctrl-a, : - open tmux command line
    * Ctrl-a, c - create window
    * Ctrl-a, p - previous window
    * Ctrl-a, b - split the current pane with a vertical line to create a horizontal layout
    * Ctrl-a, v - split the current with a horizontal line to create a vertical layout
    * Ctrl-a, <"arrow key"> - move cursor to another pane
    * Ctrl-a, Ctrl-<"arrow key"> - resize current pane

3. Midnight Commander

    * Ctrl-o - hide panels to show terminal window and reverse
    * Ctrl+Space - show directory size
    * F3 - view file using built-in viewer
    * F4 - edit file using editor of choice
    * F5 - open copy file/folder dialog
    * F6 - open move/rename file/folder dialog
    * F7 - open make folder dialog
    * F8 - open delete folder dialog
    * F9 - open pull down menu
    * F10 - exit MC


