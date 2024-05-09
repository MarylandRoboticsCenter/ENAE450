**Due date: EOD Saturday, May 18**

This is a group project. Please email me the group members and name, and I'll add it to ELMS. 

The goal of this project is to implement an algorithm in ROS2 system to autonomously navigate Turtlebot 3 Waffle Pi robot out of a maze using lidar data to avoid obstacles. The submissions will be judged based on time of completion (maze_2 time + best hardware competition run), the group with the best time will be awarded prizes. 

1. Maze Challenge: Simulation
    1. create new package and name it `maze_simulation`. You can use `hw5` package as a template
    2. add three included models (`maze_0`, `maze_1`, `maze_2`) to the new package, create corresponding `world` and launch `files`. Make sure to edit `setup.py` file accordingly
    3. in the launch files edit the spawning position to (0,0), i.e. default `x_pose` and `y_pose` should be `0`
    4. the criterion of maze completion should be the lack of obstacles on both sides of the robot based on lidar data. Upon reaching this condition, the robot should turn 180 degrees and stop. At this point the run is considered to be complete and the timer stops

2. Maze Challenge: Hardware
    1. create new package and name it `maze_hardware`
    2. the robot starts in the furtherst corner of the maze with random orientation
    3. tests will be conducted in the RAL lab (3119 IDEA Factory)
    4. the competition run
        * will be conducted on either Friday, May 17 if the majority of students are available. If not, it will be conducted on Friday, May 10
        * each group will have 3 attempts to successfully complete the maze. Each attempt will be timed, only the best time from these will be used
        * the robot completes the maze when it fully exits it, at this point the timer stops
        * hitting the walls disqualifies the attempt
        * groups should record the video of their competition runs

3. Bonus Maze Challenge: Hardware only (extra points)
    1. the robot starts on the outside wall of the maze
    2. the first goal of the robot is to find entrance to the maze and drive into it
    3. the second goal of the robot is to find a marker in the maze, drive in front of it and make a 360 degree turn
    4. hitting the walls disqualifies the attempt

**What to submit:**

1. `maze_simulation` package with proper readme file
2. `maze_hardware` package with proper readme file
3. video of `maze_2` Gazebo completion. The time of completion
4. video of best competition run. The time of completion
5. PDF of final report

**Final report:**

The final report is meant for you to showcase everything you’ve learned this semester, so use it as an opportunity to prove how much you know. Spelling and grammar matter. You may include additional sections as well, but you MUST include the following. 
* Introduction
* Hardware: Methods, Results
* Simulation: Methods, Results 
* Retrospective: What worked, what didn’t work, what you would have done differently
* Team members and how the work was divided (aka who did what)
Include any figures or snippets of code that will help to explain your process. Remember to include captions and labels. 

