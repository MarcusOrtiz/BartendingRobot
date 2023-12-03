#!/usr/bin/python3
import os
import time


class Scene(object):
    def __init__(self):
        pass

    def source_workspace(self):
        os.system("bash -c \"source $HOME/BartendingRobot/catkin_ws/devel/setup.bash\"")

    def launchGazebo(self):
        os.system("roslaunch ur5e_gripper_gazebo ur5e_gripper_bringup.launch &")
        time.sleep(5)

    def launchMoveIt(self):
        os.system(
            "roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true"
        )
        time.sleep(5)

    def sceneSetup(self):
        self.source_workspace()
        self.launchGazebo()
        self.launchMoveIt()


if __name__ == "__main__":
    scene = Scene()
    scene.sceneSetup()
