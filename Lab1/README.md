# Workflow with Docker and ROS2
1. Create a workspace folder, e.g. `ENAE450_ws` in your filesystem (not in docker container)
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
