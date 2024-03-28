**Due date: EOD Thursday, Mar 28**

**Submission guidelines:**

* create three packages (one per problem): `hw4_1`, `hw4_2`, and `hw4_3`
* all packages should be in the same workspace
* add `README.txt` file into the workspace folder (i.e. `src` and `README.txt` will be in the same folder)
* archive `src` folder and `README.txt`
    * use the following pattern to name the archive:\
     `hw4_<LastName>_<FirstName>`
    * terminal command to create archive:\
    `tar -czf hw4_<LastName>_<FirstName>.tgz src/ README.txt`
* submit the archive

**Style guidelines:**
* Make sure your code is commented, neat, and variable names make sense. 
* Consult the Python style guide https://peps.python.org/pep-0008/ as well as the ROS2 style guide: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#python. 

1. Turtlesim: Geometry figures 
    1. Use a single turtle from turtlesim to draw these geometric shapes sequentially: triangle, square, decagon, and circle (15 points)
        * the shapes should be located in different quadrants of the sqreen
        * there should be no connecting lines between the shapes
    2. Bonus (10 points):
        * use a launch file to spawn four turtles that each draw one geometric shape at the same time 

2. Turtlesim: Trajectory following
    1. Use one turtle to follow these trajectories sequentially: straight line (`y = k*x`), qubic function (`y = k*x^3`), and sine function (`y = k*sin(w*x)`) (20 points)
        * assume coord system origin is in the center of the turtlesim window
        * turtle should start at the left of the screen and go to the right
        * when reaching the end of the screen turtle should respawn on the left and start drawing the next function. Screen should be wiped between each function
        * assume `k = 1`, `w = 1`
    2. Bonus (10 points):
        * make `k` and `w` parameters.
        * add service (`func_sel`) that selects which function the turtle follows

3. Turtlesim: Turtle following
    1. Create a launch file that opens up turtlesim with a single turtle. Then, spawn new turtles, which the original turtle will go to. Once it reaches the spawned turtle, that turtle will disappear (40 points)
        * spawn several targets initially (2-4) and have a timer that spawns new goal turtles. The timer period is a parameter, set the initial value yourself.
        * create the turtle_controller node, subscribe to /turtle1/pose. Create a control loop to reach a given target. A little bit of math will be required to find the distances and angles, and send the 
        command to the /turtle1/cmd_vel topic
        * keep an array of alive turtles (name and coordinates) in the turtle_spawner node. Publish this array on the /alive_turtles topic. On the turtle_controller node, subscribe to the topic, get the array, and choose to select the first turtle in the array as the new target
        * create a service /catch_turtle in turtle_spawner. Once the turtle_controller has reached a turtle, it will send the name of the turtle to that service. From the turtle_spawner node, call the /kill service, remove the turtle from the array, and publish an updated array to /alive_turtles
        * turtle can only move forward at a constant velocity, that is `linear.x=1`, `linear.y=0`. Maximum of angular velocity is limited, that is `max angular.z=1`
    2. Bonus (5 points):
        * add an option of respawning the turtle in the middle if it hits any of the walls
    3. Bonus (10 points):
        * improve the turtle_controller to select the closest turtle instead of the first turtle in the array
    4. Bonus (30 points):
        * add random turtles that act as obstacles and not goals
        * make number of spawned obstacle turtles a parameter. Set the value of the parameter in the launch file.

