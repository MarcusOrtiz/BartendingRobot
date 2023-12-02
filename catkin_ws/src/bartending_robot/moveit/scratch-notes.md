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
                - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch/inc/load_ur5e.launch.xml
                    - `<arg name="joint_limit_params" default="$(find ur_description)/config/ur5e/joint_limits.yaml"/>`
                        - /home/jgt16/ws_moveit/src/universal_robot/ur_description/config/ur5e/joint_limits.yaml
                    - `<arg name="kinematics_params" default="$(find ur_description)/config/ur5e/default_kinematics.yaml"/>`
                        - /home/jgt16/ws_moveit/src/universal_robot/ur_description/config/ur5e/default_kinematics.yaml
                    - `<arg name="physical_params" default="$(find ur_description)/config/ur5e/physical_parameters.yaml"/>`
                        - /home/jgt16/ws_moveit/src/universal_robot/ur_description/config/ur5e/physical_parameters.yaml
                    - `<arg name="visual_params" default="$(find ur_description)/config/ur5e/visual_parameters.yaml"/>`
                        - /home/jgt16/ws_moveit/src/universal_robot/ur_description/config/ur5e/visual_parameters.yaml
                            - `mesh: "package://ur_description/meshes/ur5e/visual/base.dae"`
                                - /home/jgt16/ws_moveit/src/universal_robot/ur_description/meshes/ur5e/visual/
                            - `mesh: "package://ur_description/meshes/ur5e/collision/base.stl"`
                                - /home/jgt16/ws_moveit/src/universal_robot/ur_description/meshes/ur5e/collision/
                    - `<include file="$(dirname)/load_ur.launch.xml" pass_all_args="true">`
                        - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch/inc/load_ur.launch.xml
                            - `<param name="robot_description" command="$(find xacro)/xacro '$(find ur5e_robotiq2f140_demo)/urdf/ur.xacro'>`
                                - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/urdf/ur.xacro
                                    - `<xacro:include filename="$(find ur_gazebo)/urdf/ur_macro.xacro"/>`
                                        - /home/jgt16/ws_moveit/src/universal_robot/ur_gazebo/urdf/ur_macro.xacro
                                    - `<xacro:include filename="$(find ur5e_robotiq2f140_demo)/urdf/robotiq_arg2f_140_model_macro.xacro"/>`
                                        - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/urdf/robotiq_arg2f_140_model_macro.xacro
                                            - `<xacro:include filename="$(find ur5e_robotiq2f140_demo)/urdf/robotiq_arg2f_transmission.xacro" />`
                                                - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/urdf/robotiq_arg2f_transmission.xacro
                                            - MESHES
                                            - `<xacro:include filename="$(find ur5e_robotiq2f140_demo)/urdf/robotiq_arg2f.xacro" />`
                                                - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/urdf/robotiq_arg2f.xacro
                                                    - MESHES
            - `<arg name="controller_config_file" default="$(find ur5e_robotiq2f140_demo)/config/ur5e_controllers.yaml" doc="Config file used for defining the ROS-Control controllers."/>`
                - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/config/ur5e_controllers.yaml
            - `<include file="$(dirname)/inc/ur_control.launch.xml">`
                - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch/inc/ur_control.launch.xml






                        
    - `roslaunch ur5e_robotiq2f140_demo moveit_planning_execution.launch sim:=true`
        - /home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/launch/moveit_planning_execution.launch