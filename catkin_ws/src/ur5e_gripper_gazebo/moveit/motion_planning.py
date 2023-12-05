#!/usr/bin/env python

# Reference: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# Some functions, e.g. `all_close`, were copied from the above tutorial

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list

from gripperClass import *


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


class UR5eMoveGroupPythonInterface(object):
    def __init__(self):
        super(UR5eMoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur5e_move_group_python_interface", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
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

        # Name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        print("============ Printing robot state")
        print(robot.get_current_state())

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.eef_link = eef_link

    def go_to_joint_state(self, j0=None, j1=None, j2=None, j3=None, j4=None, j5=None):
        joint_goal = self.move_group.get_current_joint_values()
        new_joint_values = [j0, j1, j2, j3, j4, j5]

        for i in range(len(new_joint_values)):
            if new_joint_values[i] is not None:
                joint_goal[i] = new_joint_values[i]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def plan_cartesian_path(self, scale=1, x=0, y=0, z=0):
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += scale * x
        wpose.position.y += scale * y
        wpose.position.z += scale * z
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

    def pick_and_pour_right(self):
        joint_states = [
            # Physical system start state
            (tau / 4, -tau / 4, tau / 4, 0, 0, 0),
            # Initial, conservative bend that avoids singularities
            (-pi / 12, -pi / 3, pi / 3 + 0.5, 0, 0, -0.5),
            # Bend more: Get EE to a low enough height such that it can swing sideways and grab bottle
            (-pi / 12, -0.97, pi / 3 + 0.97, 0, 0, -pi / 3),
            # Swing toward bottle
            (pi / 120, -0.97, pi / 3 + 0.97, 0, 0, -pi / 3),
            # Raise bottle high enough so we don't collide with other items on the table
            (pi / 120, -pi / 3, pi / 3, 0, 0, 0),
            # Position the bottle near the cup right before pouring
            (pi / 4 + 0.2, -1.5, 1.8, 0, 0, 0),
            # Tilt the bottle so that bottle mouth is tilted downward toward the cup
            (pi / 4 + 0.2, -1.5, 1.8, 0, 0, pi / 2 - 0.2),
        ]

        for joint_state in joint_states[:4]:
            self.go_to_joint_state(joint_state)

        # TODO: close gripper

        for joint_state in joint_states[4:]:
            self.go_to_joint_state(joint_state)

        # TODO: Put some time.sleep value here so that the bottle is tilted over the cup for some time

        # Now that the liquid is poured, start to return the bottle
        for joint_state in joint_states[6:2:-1]:
            self.go_to_joint_state(joint_state)

        # TODO: open gripper

        for joint_state in joint_states[2::-1]:
            self.go_to_joint_state(joint_state)

    def pick_and_pour_left(self):
        joint_states = []


def main():
    try:
        sim_or_phys = input(
            "Are you running a simulation or physical system? \n"
            "[1] Simulation  [2] Physical : "
        )

        if sim_or_phys in ["1", "2"]:
            umg = UR5eMoveGroupPythonInterface()
            bottle_x = float(input("Bottle x coordinate: "))  # Default for testing: 0.35
            bottle_y = float(input("Bottle y coordinate: "))  # Default for testing: 0.45
            cup_x = 0
            cup_y = 0.65
            gripper = None

            if sim_or_phys == "2":  # physical
                gripper = Gripper()
                gripper.gripper_init()

            umg.pick_and_pour_right()
            umg.pick_and_pour_left()
        else:
            print("Error: Choose [1] Simulation or [2] Physical. Exiting program.")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
