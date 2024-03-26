FROM osrf/ros:humble-desktop-full
# FROM osrf/ros:humble-desktop

# Set default shell
SHELL ["/bin/bash", "-c"]

ARG USER=user
ARG UID=1000
ARG GID=1000
ARG PW=user@123

RUN groupadd -g ${GID} -o ${USER}
RUN useradd --system --create-home --home-dir /home/${USER} --shell /bin/bash --uid ${UID} -g ${GID} --groups sudo,video ${USER} && \ 
    echo "${USER}:${PW}" | chpasswd && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV QT_X11_NO_MITSHM=1 \
    USER=${USER} \
    LANG=en_US.UTF-8 \
    HOME=/home/${USER} \
    XDG_RUNTIME_DIR=/run/user/${UID} \
    TZ=America/New_York \
    NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute


USER ${USER}
WORKDIR ${HOME}
# custom Bash prompt
RUN { echo && echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \\\$ '" ; } >> .bashrc

RUN sudo mkdir -p -m 0700 /run/user/${UID} && \
    sudo chown ${USER}:${USER} /run/user/${UID}

# Basic setup
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends --allow-unauthenticated \
    software-properties-common \
    build-essential \
    curl \
    g++ \
    git \
    ca-certificates \
    make \
    cmake \
    automake \
    autoconf \
    bash-completion \
    iproute2 \
    iputils-ping \
    libtool \
    pkg-config \
    libxext-dev \
    libx11-dev \
    mc \
    mesa-utils \
    nano \
    tmux \
    tzdata \
    xclip \
    x11proto-gl-dev && \
    sudo rm -rf /var/lib/apt/lists/*

# Setup tmux config
ADD --chown=${USER}:${USER} https://raw.githubusercontent.com/MarylandRoboticsCenter/ENAE450/main/misc/.tmux.conf $HOME/.tmux.conf

# Set datetime and timezone correctly
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | sudo tee -a /etc/timezone

# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-dev-tools \
    python-is-python3 \
    python3-pip && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

RUN pip3 install setuptools==58.2.0

# Install TB3 ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
	ros-humble-dynamixel-sdk \
	ros-humble-turtlebot3-msgs \
	ros-humble-turtlebot3 && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*


# Set up TB3 workspace
RUN source /opt/ros/humble/setup.bash && \
	mkdir -p $HOME/tb3_ws/src && \
    cd $HOME/tb3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    cd $HOME/tb3_ws && \
	colcon build --symlink-install


# Setup ROS workspace directory
RUN mkdir -p $HOME/ENAE450_ws/src    


# Set up working directory and bashrc
WORKDIR ${HOME}/ENAE450_ws/
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc && \
    echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> $HOME/.bashrc && \
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> $HOME/.bashrc && \
    echo 'export ROS_DOMAIN_ID=1' >> $HOME/.bashrc && \
    echo 'export ROS_LOCALHOST_ONLY=1' >> $HOME/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> $HOME/.bashrc && \
    echo '#source $HOME/tb3_ws/install/setup.bash' >> $HOME/.bashrc && \
    echo '#source /usr/share/gazebo/setup.bash' >> $HOME/.bashrc
    
CMD /bin/bash
