To preview the robot visually in rviz, you can make navigate to the catkin environment and
run the following commands
```
catkin_make
source devel/setup.bash
roslaunch bartending_robot display.launch
```

To run the motion planning script,

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

3. Open a third terminal tab and run
```
catkin_ws/src/ur5e_gripper_gazebo/moveit
python3 motion_planning.py
```

NOTE: We do not know yet how to pick up bottles in simulation.