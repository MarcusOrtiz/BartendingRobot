#!/bin/bash

# Victor Xia, 11/15/2022
# Object to setup experiment 
# (table size, robot locations, target locations, etc)

import os
from time import sleep

default_TaskSpace_length=2
default_TaskSpace_width=1
default_TaskSpace_height=1



'''===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v==='''
class Scene(object):
    '''
    A collection of parameters and methods to setup the experiment table.
    Also initialize Gazebo & MoveIt

    Attributes:
        x_limit  --- task space x limit, [-xlim,xlim], xlim_default=1
        y_limit  --- task space y limit, [-ylim,ylim], ylim_default=0.5
        z_limit  --- task space z limit, [0,zlim], zlim_default=1
    
    methods:
        launchGazebo()
        launchMoveIt()
        launchRViz()
        generateCup()
        generateBottle()
        sceneSetup() --- a pipeline function that calls others to setup the default scene
    ''' 

    def __init__(self, TaskSpace_length=default_TaskSpace_length, TaskSpace_width=default_TaskSpace_width, TaskSpace_height=default_TaskSpace_height):
        '''
        Create a task space (table) with the robot arm at the center.
        Default length=2 (-1<x<1), width=1 (-0.5<y<0.5)
        '''
        self.x_limit=[-TaskSpace_length/2,TaskSpace_length/2]
        self.y_limit=[-TaskSpace_width/2,TaskSpace_width/2]
        self.z_limit=[0,TaskSpace_height/2]
        pass

    def launchGazebo(self):
        '''
        roslaunch ur_gazebo ur5.launch limited:=true
        '''
        cmd='roslaunch ur_gazebo ur5.launch limited:=true >/dev/null &'
        os.system(cmd)
        sleep(5)
        pass

    def launchMoveIt(self):
        '''
        roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch 
        sim:=true limited:=true
        '''
        cmd='roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch '\
        'sim:=true limited:=true >/dev/null &'
        os.system(cmd)
        sleep(3)
        pass

    def launchRViz(self):
        '''
        roslaunch ur5_moveit_config moveit_rviz.launch config:=true
        '''
        cmd='roslaunch ur5_moveit_config moveit_rviz.launch config:=true >/dev/null &'
        os.system(cmd)
        sleep(3)
        pass

    def generateCup(self, x=0, y=0.5, z=0):
        '''
        Create a cup-shaped object and put at (x=0, y=0.5, z=0)
        '''
        spawn_command='rosrun gazebo_ros spawn_model -file ./Scene/cup/model.sdf -sdf -x {} -y {} -z {} -model cup >/dev/null &'.format(x,y,z)
        os.system(spawn_command)
        sleep(1)
        pass

    def generateBottle(self, x=0, y=-0.5, z=0):
        '''
        Create a bottle-shaped object and put at (x=0, y=-0.5, z=0)
        '''
        spawn_command='rosrun gazebo_ros spawn_model -file ./Scene/bottle/model.sdf -sdf -x {} -y {} -z {} -model bottle >/dev/null &'.format(x,y,z)
        os.system(spawn_command)
        sleep(1)
        pass

    def sceneSetup(self):
        '''
        a pipeline function that calls others to setup the default scene
        '''
        self.launchGazebo()
        self.launchMoveIt()
        # self.launchMoveIt() # no need for RViz
        self.generateCup()
        self.generateBottle()
        pass


'''===^===^===^===^===^===^===^===^===^===^===^===^===^===^===^==='''
def main():
    '''Unit test code'''
    scene=Scene()
    scene.sceneSetup()
    pass

if __name__ == '__main__':
    main()
    pass
    