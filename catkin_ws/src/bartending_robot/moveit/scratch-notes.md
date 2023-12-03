# top to bottom references
/home/jgt16/BartendingRobot/catkin_ws/src/ur5e_gripper_gazebo/
- `launch.py`
    - `launch/ur5e_gripper_bringup.launch`
        - `launch/inc/load_ur5e_gripper.launch.xml`
            - `launch/inc/load_ur.launch.xml`
                - `urdf/ur5e_gripper.xacro`
                    - `urdf/ur_macro.xacro`
                    - `urdf/gripper.xacro`
                        - MESHES
                    - `urdf/bottle.xacro`
                    - `urdf/cup.xacro`
                    - `urdf/platform.xacro`
        - `config/ur5e_gripper_controllers.yaml`
        - `launch/inc/ur_control.launch.xml`
    - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/launch/moveit_planning_execution.launch`
        - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/launch/move_group.launch`
            - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/launch/planning_context.launch`
                - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/config/ur5e.srdf`
                - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/config/joint_limits.yaml`
                - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/config/kinematics.yaml`
            - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/launch/trajectory_execution.launch.xml`
                - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/launch/ur5e_moveit_controller_manager.launch.xml`
                    - `~/ws_moveit/src/universal_robot/ur5e_moveit_config/config/ros_controllers.yaml`


# Of Jonathan's demo files, inspected these:
/home/jgt16/catkin_ws/src/ur5e_robotiq2f140_demo/
- `launch.py` # similar
    - `launch/ur5e_bringup.launch` # COPIED TO FINAL, similar except for some paths and "gripper"
        - `launch/inc/load_ur5e.launch.xml` # LEFT AS IS, similar
            - `launch/inc/load_ur.launch.xml` # LEFT AS IS, similar
                - `urdf/ur.xacro` # BIG DIFF
                    - `urdf/robotiq_arg2f_140_model_macro.xacro`
                        - `urdf/robotiq_arg2f_transmission.xacro`
                        - MESHES
                        - `urdf/robotiq_arg2f.xacro`
                            - MESHES
        - `config/ur5e_controllers.yaml` # COPIED TO FINAL; there are additions here
        - `launch/inc/ur_control.launch.xml` # no difference, no action needed 
    - `launch/moveit_planning_execution.launch` # COPIED TO FINAL; similar to ws_moveit except for path
        - `launch/move_group.launch` # COPIED TO FINAL; similar to ws_moveit except for paths
            - `launch/planning_context.launch` # COPIED TO FINAL; similar except for paths
                - `urdf/ur5e_robot.srdf` # compare to ~/ws_moveit/src/universal_robot/ur5e_moveit_config/config/ur5e.srdf # COPIED TO FINAL (BUT STILL NEEDS COMPARISON AND EDITS) VERY DIFFERENT
                - `config/joint_limits.yaml` # COPIED TO FINAL; somewhat different
                - `config/kinematics.yaml` # COPIED TO FINAL; somewhat different
            - `launch/trajectory_execution.launch.xml` # COPIED TO FINAL; similar except for path
                - `launch/ur5e_moveit_controller_manager.launch.xml` # COPIED TO FINAL; similar except for path
                    - `config/ros_controllers.yaml` # COPIED TO FINAL; there are additions here