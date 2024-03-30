ENAE450 HW3 solution

`src` folder contains 3 packages: 
    - hw3_1_sol: solution for hw3_1
    - hw3_2_sol: solution for hw3_2
    - interface_package: custom msg and srv required for hw3

Instructions:

1. Build the `src` folder using colcon:

    colcon build --symlink-install

2. Source the built packages

    source install/setup.bash

3. open `tmux` and create several panes. Two would be required to run the code and others can be useful for running ROS cli commands
4. to run hw3_1_sol:
    in first tmux pane execute:

        ros2 run hw3_1_sol hw3_1_pub

    in second tmux pane execute:

        ros2 run hw3_1_sol hw3_1_sub

5. to run hw3_2_sol:
    in first tmux pane execute:

        ros2 run hw3_2_sol hw3_2_pub

    in second tmux pane execute:

        ros2 run hw3_2_sol hw3_2_sub

5. to run hw3_2_sol for bonus points:
    in first tmux pane execute:

        ros2 run hw3_2_sol hw3_2_pub_bonus

    in second tmux pane execute:

        ros2 run hw3_2_sol hw3_2_sub_bonus