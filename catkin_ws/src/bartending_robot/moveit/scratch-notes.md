# top to bottom references

- /home/jgt16/BartendingRobot/catkin_ws/src/ur5e_gripper_gazebo/launch/ur5e_gripper_bringup.launch
- /home/jgt16/BartendingRobot/catkin_ws/src/ur5e_gripper_gazebo/launch/inc/load_ur5e_gripper.launch.xml
- /home/jgt16/BartendingRobot/catkin_ws/src/ur5e_gripper_gazebo/launch/inc/load_ur.launch.xml
- /home/jgt16/BartendingRobot/catkin_ws/src/ur5e_gripper_gazebo/urdf/ur5e_gripper.xacro
    - ur_macro.xacro
        - ur.xacro
    - gripper.xacro

DO NOT EDIT /home/jgt16/BartendingRobot/catkin_ws/src/ur5e_gripper_gazebo/urdf/ur.xacro

# Of Jonathan's demo files, inspected these:
- /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch.py
    - `source $HOME/ws_moveit/devel/setup.bash`
    - `source $HOME/catkin_ws/devel/setup.bash`
    - `cd $HOME/catkin_ws/src`
    - `roslaunch ur5e_robotiq2f140_demo ur5e_bringup.launch`
        - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch/ur5e_bringup.launch
            - `<arg name="robot_description_file" default="$(dirname)/inc/load_ur5e.launch.xml" doc="Launch file which populates the 'robot_description' parameter."/>`
    - `roslaunch ur5e_robotiq2f140_demo moveit_planning_execution.launch sim:=true`
        - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch/moveit_planning_execution.launch