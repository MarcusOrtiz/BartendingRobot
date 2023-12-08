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
# This code is adapted from the tutorial posted on the Ed Discussion

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input



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
        print("Welcome to the Motion Planning to Reach the Left Bottle")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin ..."
        )
        tutorial = jasmineinitial()

        input(
            "============ Press `Enter` to use joint state goal to go to starting position..."
        )
        tutorial.go_to_joint_state(0, -tau/4, tau/4, 0, 0, 0)

        input(
            "============ Press `Enter` to use joint state goal to position the end effector above the cup..."
        )
        tutorial.go_to_joint_state(0.785398, -tau/4, tau/4, 0, 2.5346, 0)
        

        
<<<<<<< HEAD
        input("============ Press `Enter` to move down in order to grasp the bottle ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.3) 
=======
        input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.505) 
>>>>>>> 9023918802cc3a58531390c0381276fac196ee5f


        input("============ Press `Enter` to execute the saved path and also to get the current joint values...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())

        input("============ Press `Enter` to raise the cup up...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, 0.2) 


        input("============ Press `Enter` to execute a saved path and also to get the current joint values...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())


        input(
            "============ Press `Enter` to use joint state goal to go to a position adjacent to the cup..."
        )
        tutorial.go_to_joint_state(0.38608914539012, -1.536518252192991, 1.818695914524728, -0.27412769104616384, 2.5344015241084747, 9.515101281998284e-05) 


        input("============ Press `Enter` to move down to be closer to the edge of the cup...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.2) 


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())

        
        input(
            "============ Press `Enter` to rotate the wrist 135 degrees in order to pour the M&Ms..."
        )
        tutorial.go_to_joint_state(0.3866538305475169, -1.2583514477007052, 2.1079269923743045, -0.804250570600197, 2.534104647576573, 2.35619) 

        input(
            "============ Press `Enter` to return the wrist joint to 0 degrees..."
        )
        tutorial.go_to_joint_state(0.3866538305475169, -1.2583514477007052, 2.1079269923743045, -0.804250570600197, 2.534104647576573, 0)

       

        input(
            "============ Press `Enter` to use joint state goal to go to the bottle's original position..."
        )
<<<<<<< HEAD
        tutorial.go_to_joint_state(0.785398, -tau/4, tau/4, 0, 2.5346, 0) 

        input("============ Press `Enter` to move down in order to set the bottle down...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.2) 
=======
        tutorial.go_to_joint_state(0.785398, -tau/4, tau/4, 0, 2.5346, 0)

        input("============ Press `Enter` to plan and display a Cartesian path to draw the final line of k...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, -0.505) 
>>>>>>> 9023918802cc3a58531390c0381276fac196ee5f


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())

        input("============ Press `Enter` to move up and out of the bottle's way...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0, 0, 0.35) 


        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)
        print(tutorial.move_group.get_current_joint_values())

        input(
            "============ Press `Enter` to return to the starting position..."
        )
        tutorial.go_to_joint_state(0, -tau/4, tau/4, 0, 0, 0)

        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return    


if __name__ == "__main__":
    main()

