#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
# from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachModelResponse
# from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
# from copy import deepcopy
# from tf.transformations import quaternion_from_euler


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


import math
from time import sleep

try:
    from math import pi, tau, dist, fabs, cos
except:  
    # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    # tau is one full revolution for a joint, which is referenced later in
    #   the joint goals function
    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True
class jasmineinitial(object):
    """jasmineinitiall"""

    def __init__(self):
        super(jasmineinitial, self).__init__()

        ## begin setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("jasmineinitial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  Group name is changed to manipulator
        ## because this is a UR5e. 
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
     
    #create a function that can allow robot to return to the same position at the beginning
    #of each initial; numbers chosen from joints in Rviz 
    def go_to_joint_state(self, j1, j2, j3, j4, j5, j6):
    
        move_group = self.move_group
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## Want to move the robot arm out of a singularity so use joint goals to
        ## make a starting position
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j1
        joint_goal[1] = j2 
        joint_goal[2] = j3
        joint_goal[3] = j4
        joint_goal[4] = j5
        joint_goal[5] = j6 
    

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        print('Movement Done')

        # rospy.loginfo("Attaching greenbottle and hand")
        # amsg = Attach()
        # amsg.model_name_1 = "Green_bottle"
        # amsg.link_name_1 = "wrist_3_link"

     #use cartesian points to actually move the robot in directions to trace letters
    def plan_cartesian_path(self, wx, wy, wz, scale=1):
       
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ## create an empty array for waypoints
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x += scale * wx
        wpose.position.y += scale * wy
        wpose.position.z += scale * wz
        waypoints.append(copy.deepcopy(wpose))
        

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
       
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

    #     ## **Note:** The robot's current joint state must be within some tolerance of the
    #     ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    #     ## END_SUB_TUTORIAL

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = jasmineinitial()

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(tau/4, -tau/4, tau/4, 0, 0, 0)

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(-3.14159, -tau/4, tau/4, 0, 0, 0)

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(-3.24159, -tau/4, tau/4, 0, 0.9908, 0)

        
        input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0.2, 0, 0)


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)
        
        input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.3) #go back to -0.4
        


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())

        # #for 45, 25
        # input(
        #     "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        # )
        # tutorial.go_to_joint_state(3.101079480209145, -1.6635704675457772, 2.5551133085608004, -0.8722878098566937, 1.0904281488791465, 0.0010008298702386398) 
        

        input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(-0.1, 0, 0)
        print(tutorial.move_group.get_current_joint_values())


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)

        
        input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, 0.3) 


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())


        # input(
        #     "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        # )
        # tutorial.go_to_joint_state(2.800977698508065, -1.9077536886361264, 1.9988316077204686, -0.06159882871521649, 1.0312317631021628, -0.001552384283574959) 

        # input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(-0.25, 0, 0) 
        # print(tutorial.move_group.get_current_joint_values())

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.20) 
        # print(tutorial.move_group.get_current_joint_values())

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)
        # print(tutorial.move_group.get_current_joint_values())

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(-2.968708305419426, -1.1016912351673032, 1.708198415385449, -0.554833237377963, 1.1989676922643264, 0) 

        
        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(-2.968708305419426, -1.1016912351673032, 1.708198415385449, -0.554833237377963, 1.1989676922643264, 2.0944) 

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(-2.968708305419426, -1.1016912351673032, 1.708198415385449, -0.554833237377963, 1.1989676922643264, 0)

        #  #for 45, 25
        # input(
        #     "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        # )
        # tutorial.go_to_joint_state(3.101079480209145, -1.6635704675457772, 2.5551133085608004, -0.8722878098566937, 1.0904281488791465, 0.0010008298702386398) 
        

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(-3.5049831635030326, -1.9034665279362724, 2.7113104246246014, -0.7903283035591855, 1.2540743561747494, 0.0021673398748349726) 

        input(
            "============ Press `Enter` to use joint state goal to go to starting position for first letter..."
        )
        tutorial.go_to_joint_state(tau/4, -tau/4, tau/4, 0, 0, 0)

        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return    


if __name__ == "__main__":
    main()

