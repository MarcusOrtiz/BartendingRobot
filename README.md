To preview the robot visually in rviz, you can make navigate to the catkin environment and
run the following commands
```
catkin_make
source devel/setup.bash
roslaunch bartending_robot display.launch
```

# Motion Planning testing (Dec 5)

We currently have two (2) motion planning scripts. Why? There's one for picking a bottle up from the right hand side, and one for the left. Still WIP! They need to be combined into one.

In the meantime, here's how to run them:

1. Open a terminal tab and run
```
cd catkin_ws
source devel/setup.bash
make bringup
```

2. Open a second terminal tab and run
```
source devel/setup.bash
make execution
```

3. To run the script that picks a bottle up from the **right** hand side,
    - Right hand side
    ```
    catkin_ws/src/ur5e_gripper_gazebo/moveit
    python3 motion_planning_right.py
    ```
    - Left hand side
    ```
    catkin_ws/src/ur5e_gripper_gazebo/moveit
    python3 motion_planning_left.py
    ```

NOTE: We do not know yet how to pick up bottles in simulation.