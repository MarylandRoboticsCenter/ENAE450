# General overview
The goal of this lab is to install ROS2 Humble. The final test of the installation will be running terminal commands such as
```bash
glxgears
```
or
```bash
ign gazebo empty.sdf
```
and making sure the proper graphical output is displayed.

**Note:** Just to be clear this is not the only way to install ROS2 Humble. The experienced or adventerous students can install ROS without any virtualization software by following insructions available online (e.g. [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)). In this case students are responsible for making sure ROS and the required packages operate properly.

# PC with Ubuntu 20.04 and later, installation steps with Docker
This is a simplified visualization of what we are trying to achive:\
OS => Docker (virtualization software) => Ubuntu 22.04 + ROS2 Humble (container)\
It doesn't matter if you launching Ubuntu using dual booting, or single booting, or using VM software. 

1. Installing Docker using apt repository ([full guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository))
    * Uninstall old Docker versions
    * Set up Docker's apt repository
    ```bash
    # Add Docker's official GPG key:
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    ```
    * Install the Docker packages
    ```bash
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```
    * Verify installation success
    ```bash
    sudo docker run hello-world
    ```
    * Change permissions to run docker without sudo
    ```bash
    sudo usermod -aG docker $USER
    ```
    * Restart PC
    * Run the following and make sure there is no error
    ```bash
    docker ps -a
    ```
2. Installing Nvidia drivers and Docker support (**only if you have Nvidia GPU**)
    * Test if you have Nvidia drivers installed already
    ```bash
    nvidia-smi
    ```
    * If you get error consider installing the drivers yourself (see [link](https://ubuntu.com/server/docs/nvidia-drivers-installation)) or talk to your insructor
    * If `nvidia-smi` returned proper output, then install Nvidia container toolkit (see full guide [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))
    ```bash
    # one large command
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    # command ended
    
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```
3. Downloading and building Docker image
    * Download Dockerfils and navigate terminal to the folder it is stored
    * Run the following command (make sure to include "." in the last line)
    ```bash
    DOCKER_BUILDKIT=1 docker build --build-arg USER=$USER \
        --build-arg UID=$(id -u) \
        --build-arg GID=$(id -g) \
        --build-arg PW=docker \
        -t tb3_image \
        -f humble_dockerfile.Dockerfile\
        .
    ```
    * For the class purposes I used the abridged version of the docker image (see first line of the Docker file). Outside of the class please delete the first line and uncomment the second line (FROM osrf/ros:humble-desktop-full)
4. Running Docker container
    * If you have Nvidia GPU, run the following command
    ```bash
    docker run -it --rm --name TB3Container --net=host --ipc=host --pid=host --gpus=all --runtime=nvidia --privileged \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    tb3_image:latest
    ```
    * If you **don't** have Nvidia GPU, run the following command
    ```bash
    docker run -it --rm --name TB3Container --net=host --ipc=host --pid=host --privileged \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    tb3_image:latest
    ```
# PC with Ubuntu 22.04, installation steps without Docker
1. Make sure your system is up to date
    ```bash
    sudo apt-get update
    sudo apt-get dist-upgrade
    ```

2. Install supplementary packages
    ```bash
    sudo apt-get update
    sudo apt-get install -y build-essential curl git make cmake iproute2 iputils-ping mc mesa-utils nano tmux 
    ```

3. Install ROS2 Humble 

    Follow official instructions from [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

3. Install supplementary ROS packages
    ```bash
    sudo apt-get install -y ros-dev-tools python-is-python3 python3-pip
    ```

4. Fix python `setuptools` package
    ```bash
    pip3 install setuptools==58.2.0
    ```

5. Install TB3 ROS packages
    ```bash
    sudo apt-get install -y ros-humble-gazebo-* ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2  ros-humble-nav2-bringup ros-humble-dynamixel-sdk ros-humble-turtlebot3-msgs ros-humble-turtlebot3
    ```

5. Set up TB3 workspace
    ```bash
    source /opt/ros/humble/setup.bash
    mkdir -p $HOME/tb3_ws/src
    cd $HOME/tb3_ws/src
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd $HOME/tb3_ws
    colcon build --symlink-install
    ```

5. Source your ROS2 installation and `colcon` autocomplete
    ```bash
    echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc
    echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> $HOME/.bashrc
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> $HOME/.bashrc
    echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc
    echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> $HOME/.bashrc
    echo '#source $HOME/tb3_ws/install/setup.bash' >> $HOME/.bashrc
    echo '#source /usr/share/gazebo/setup.bash' >> $HOME/.bashrc
    ```

4. (Optionally but recommended) Download tmux config file
    ```bash
    wget https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.bash.conf -O $HOME/.tmux.conf
    ```    

# PC with Windows 10 and later, installation steps with Docker
On Windows machines our setup will have another virtualization layer but the rest will repeat Ubuntu setup
Windows 10 => WSL2 with Ubuntu 20.04 (virtualization software) => Docker (virtualization software) => Ubuntu 22.04 + ROS2 Humble (container)
1. Installing WSL2 (see full guides [1](https://linuxbeast.com/devops/how-to-install-ubuntu-20-04-in-wsl2-on-a-windows-10/), [2](https://learn.microsoft.com/en-us/windows/wsl/install-manual))
    * Run Powershell as Administrator
    * Verify compatibility of your system
    ```bash
    systeminfo | findstr /B /C:"OS Name" /C:"OS Version"
    ```
    * Enable Windows Machine Platform
    ```bash
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
    ```
    Set WSL2 as the default version for future WSL installations
    ```bash
    * wsl --set-default-version 2
    ```
    * Install Ubuntu 20.04 in WSL 2
    ```bash
    wsl --install -d Ubuntu-20.04
    ```
    * To launch WSL2, start Powershell in a regular mode and run
    ```bash
    wsl
    ```
2. Follow instructions from ["PC with Ubuntu 20.04 and later ..."](#pc-with-ubuntu-2004-and-later-installation-steps-with-docker). All commands should be executed from WSL2 environment

# Mac laptops
There are some issues with GPU acceleration support for Docker applications running on Mac. Mac users should use VMware Fusion virtualization software instead. This is a simplified visualization of what we are trying to achive:\
Mac OS => VMware Fusion (virtualization software) => Ubuntu 22.04 + ROS2 Humble (container)\
Since VMware Fusion can be taxing on resources of Mac laptops, we won't use Docker as a middle virtualization layer. Instead 

1. Download VMware Fusion 13. Use *one* of these options:
    * Follow UMD Terpware [instructions](https://terpware.umd.edu/Mac/title/4092)
    * Download VMware Fusion Tech Preview 2023 as described [here](https://blogs.vmware.com/teamfusion/2023/07/vmware-fusion-2023-tech-preview.html)
2. Install VMware Fusion 13 (Tech Preview) on your Mac laptop
3. Download Ubuntu 22.04 desktop image
    * For older Macs download **amd64** image from [here](https://releases.ubuntu.com/jammy/)
    * For Macs with M1/M2 chips download **arm64** image from [here](https://cdimage.ubuntu.com/jammy/daily-live/current/)
4. Use the downloaded Ubuntu image to create a new Virtuale Machine
5. Start the Virtuale Machine and follow instructions from ["PC with Ubuntu 22.04 ..."](#pc-with-ubuntu-2204-installation-steps-without-docker)

# Installation test
1. Start Docker container using image that you built (`tb3_image`).
2. Verify that ROS2 environmental variables are set
```bash
    env | grep ROS
```
3. Verify that graphics output is working (you should see a popup window with rotating gears)
```bash
    glxgears
```
4. Verify that Gazebo is installed (you should see empty Gazebo environment)
```bash
    ign gazebo empty.sdf
```

# IDE installation (optional)
Install your IDE of choice, e.g. [VS Code](https://code.visualstudio.com/download)

# Troubleshooting 

## General
1. Tmux shortcuts don't work (e.g. when trying to create new panes)
    * If you are using setup with Docker, make sure that you copied the tmux config file into your home folder
    ```bash
    wget https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.bash.conf -O $HOME/.tmux.conf
    ```
    * If you are using setup with Docker, talk to your instructor. Perhaps you are entering the shortcuts incorrectly

2. Problems with listing ROS2 nodes
    * Restart your ROS2 daemon
    ```bash
    ros2 daemon stop
    ros2 daemon start
    ```

## Windows PC
1. Issues with connecting to Docker daemon but `docker version` works properly. Try starting docker service manually, in the WSL2 terminal run
    ```bash
    sudo service docker start
    ```
    If it works, you'll have to run this command every time you launch WSL2 terminal. There is a workaround that we can discuss deparately.

2. When launching GUI apps in docker, it throws an error because it cannot access DISPLAY variable.
    * If running Windows 10 make sure the OS build version is above 19044. If not, run the system update.
    * In the Powershell run
    ```bash
    wsl --update
    ```
    * If the issue persists, check if you have the GPU drivers installed, see [here](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)


# Suggestion (Windows PC)
1. Remove path to windows binaries in WSL2
    * Edit /etc/wsl.conf, e.g.
    ```bash
        sudo nano /etc/wsl.conf
    ```
    * Add the following to the file
    ```bash
        [interop]
        appendWindowsPath = false
    ```
    * Restart WSL
