#!/usr/bin/python3
import os
import time
from pathlib import Path

class Scene(object):
    def __init__(self):
        pass
    
    def setup_catkin(self):
        curr_dir = Path.cwd().absolute()  # looks redundant, but I'm just initializing

        if "catkin_ws" in curr_dir.parts:
            while "catkin_ws" in curr_dir.parts and "catkin_ws" != curr_dir.stem:
                print(f"Have not arrived at `catkin_ws` yet. Currently at {Path.cwd()}")
                os.chdir(Path.cwd().parent)
                curr_dir = Path.cwd().absolute()
            print(f"Arrived at {curr_dir}")
        else:
            print("Error: `catkin_ws` not found.")

        os.system("catkin_make &")
        time.sleep(5)
        os.system("source devel/setup.bash &")
        time.sleep(5)


    def launchGazebo(self):
        os.system("roslaunch ur5e_gripper_gazebo ur5e_gripper_bringup.launch &")
        time.sleep(5)

    def launchMoveIt(self):
        os.system(
            "roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true"
        )
        time.sleep(5)

    def sceneSetup(self):
        self.setup_catkin()
        self.launchGazebo()
        self.launchMoveIt()


if __name__ == "__main__":
    scene = Scene()
    scene.sceneSetup()

