#!/usr/bin/python3
import os


class Scene(object):
    def __init__(self):
        pass

    def sceneSetup(self):
        commands = [
            "roslaunch ur5e_gripper_gazebo ur5e_gripper_bringup.launch",
            "sleep 10 ; roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true",
        ]

        for command in commands:
            os.system(
                f'bash -c "source $HOME/BartendingRobot/catkin_ws/devel/setup.bash ; {command} &"'
            )


if __name__ == "__main__":
    scene = Scene()
    scene.sceneSetup()
