**Due date: EOD Tuesday, Apr 23**

**Submission guidelines:**

* for problems #2 and #3 use `hw5` package which is located in this folder
* create separate launch files for each problems #2 and #3
* add `README.txt` file into the workspace folder (i.e. `src` and `README.txt` will be in the same folder)
* archive `src` folder and `README.txt`
    * use the following pattern to name the archive:\
     `hw5_<LastName>_<FirstName>`
    * terminal command to create archive:\
    `tar -czf hw5_<LastName>_<FirstName>.tgz src/ README.txt`
* submit the archive
* submit the videos along with archive


**Style guidelines:**
* Make sure your code is commented, neat, and variable names make sense. 
* Consult the Python style guide https://peps.python.org/pep-0008/ as well as the ROS2 style guide: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#python. 

1. SLAM video recording
    1. Use Gazebo simulation of [Turtlebot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) to generate and save a map of TurtleBot3 House demo (10 points)
        * drive around to get a decent coverage of the majority of the map
        * record a video of the process starting with launching ROS2 nodes in the terminal
        * video recording has to have good enough quality so that the terminal commands are legible. Preferably use screen grabbing software
    2. Use Gazebo simulation of [Turtlebot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/) and the generated map to do the localization demo (10 points)
        * perform initial pose estimation
        * manually drive around until the pose estimation converges to a small area under the Turtlebot
        * set Navigation Goal to generate a path and automatically follow it
        * record a video of the process starting with launching ROS2 nodes in the terminal
    3. Bonus (10 points): use the following (and any additional) materials to write an explanation of how the SLAM alorithm in the demo works
        * [(SLAM)-An Overview](https://www.jastt.org/index.php/jasttpath/article/view/117/37)
        * [ROSCon 2019 Macau: On Use of SLAM Toolbox](https://vimeo.com/378682207)
        * [Understanding the Particle Filter](https://www.youtube.com/watch?v=NrzmH_yerBU)
        * [Understanding SLAM Using Pose Graph Optimization](https://www.youtube.com/watch?v=saVZtgPyyJQ)
        * [Why SLAM Matters](https://www.mathworks.com/discovery/slam.html#:~:text=SLAM%20(simultaneous%20localization%20and%20mapping)%20is%20a%20method%20used%20for,to%20map%20out%20unknown%20environments.)

2. Turtbot: Inside wall following (25 points)
    1. Write ROS code to control Gazebo simulation of Turtlebot3 in a way that it is driving on the inside perimeter of rectangular enclosure
        * use `hw5` package and `tb3_4walls_inside.launch.py` launch file to start the Gazebo simulation
        * use lidar data (`scan` topic) to obtain measurement of the distance to the walls
        * drive TB3 to a distance of `0.5m` from the wall, turn right and continue moving clockwise following the wall at the same distance
    2. CHEATING (-10 points):
        * obtain `pose` estimation by subscribing to `tf` or `odom` topics or using any other methods
    3. Bonus (7 points):
        * implement two parameters (initiate them in the `launch` file) to set travel distance from the wall and clockwise/counterclockwise movement

3. (Bonus problem) Turtbot: Outside wall following (20 points)
    1. Write ROS code to control Gazebo simulation of Turtlebot3 in a way that it is driving on the outside perimeter of rectangular enclosure
        * use `hw5` package (in `homework` folder) and `tb3_4walls_outside.launch.py` launch file to start the Gazebo simulation
        * use lidar data (`scan` topic) to obtain measurement of the distance to the walls
        * drive TB3 to a distance of `0.5m` from the wall, turn right and continue moving clockwise following the wall at the same distance
        
    2. CHEATING (don't do it):
        * obtain `pose` estimation by subscribing to `tf` or `odom` topics or using any other methods

