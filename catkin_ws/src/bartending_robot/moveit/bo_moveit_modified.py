#!/usr/bin/env python

# IMPORTS/PACKAGES USED IN TUTORIAL, 
# DID NOT WANT TO BREAK ANY PIECES OF THE CODE
# SO KEPT MOST JUST TO BE SAFE
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gripperClass import *

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
pass


'''===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v==='''
class MoveGroupUR5PythonInterface(object):
    """
    MoveGroupUR5PythonInterface



    Attributes:
        box_name
        robot
        scene
        move_group
        display_trajectory_publisher    <-- ROS publisher to display trajectories in Rviz

        d1  <-- shoulder_height, type=float, unit=m
        a2  <-- upper_arm_length, type=float, unit=m
        a3  <-- forearm_length, type=float, unit=m
        d4  <-- wrist_1_length, type=float, unit=m
        d5  <-- wrist_2_length, type=float, unit=m
        d6  <-- wrist_3_length, type=float, unit=m

        platform_dist   <-- PLATFORM DISTANCE FROM BASE OF ROBOT, with clearance, type=float, unit=m
        cup_dist        <-- CUP DISTANCE FROM BASE OF ROBOT, with clearance, type=float, unit=m

        int_obstacle_lower  <-- LOWER BOUND, CLOSEST INTERIOR OBSTACLE RADIALLY WITHIN TASKSPACE,
                                accomodates radius of robot arm and metal pedestal, type=float, unit=m
        int_obstacle_perp   <-- PERP BOUND, type=float, unit=m
        int_obstacle_upper  <-- HIGHER BOUND, type=float, unit=m

        platform_loc    <-- indicates location of platform in space, type=float, unit=rad

        init_shoulder_angle <-- INIT SHOULDER ANGLE, type=float, unit=deg

        ee_to_gripper   <-- EE TO GRIPPER CENTER DISTANCES, type=float
        ee_to_origin    <-- EE TO origin DISTANCES, type=float

        initial Initial Variable    <-- indicates picking up or pouring stage, type=bool

        gripper_to_cup  <-- Gripper to cup distance on x-y plane, unit=m



    Methods:
        law_of_cosines()    <-- Solve triangle interior angles using three sides.

        compute_future_joint_state()    <-- Compute the joint state it need to be in to pick-up/pour.
                                            Returns joint values shoulder_joint, elbow_joint, wrist_1_joint.

        go_to_radial_joint_state()  <-- Point radially at the target and get ready to pick-up/pour

        init_rotational_position_turn() <-- Calculate joints 2/3/4 angles necessary for radial translation, 
                                            and move.

        init_rotational_position()  <-- Turn the gripper away from the wrist and stay clear from during the movement

        compute_shoulder_rotation() <-- Comupte the shoulder joint value it need to be at.
                                        Return future shoulder joint angle

        move_to_target()    <-- Calculate joint 1 angle necessary to move clockwise to target, 
                                and move.

        lift_bottle()

        find_shoulder_angle_change()    <-- Find the shoulder joint angle it need to change to pick-up/puor the bottle
        
        pour_bottle()

        perform_first_phase()   <-- Going from unknown state, move to init pos.
                                    Move to bottle with open gripper

        perform_second_phase()  <-- Once grip bottle, lift and return to init pos.

        perform_third_phase()   <-- After receive x,y coords of glass, move to target,
                                    sleep for pouring, return to init state
    """
    def __init__(self):
        '''
        Sets up our move_it commander and our particular robot used (URe5 "manipulator"),
        also setup the experiment environment parameters
        '''

        super(MoveGroupUR5PythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur5_python_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        # group_name is the particular robotic arm used
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        #LINK LENGTHS EXTRACTED FROM URDF FILE
        d1 = 0.08916 #shoulder_height
        a2 = 0.4250 # upper_arm_length
        a3 = 0.39225 # forearm_length
        d4 = 0.10915 # wrist_1_length
        d5 = 0.09465 # wrist_2_length
        d6 = 0.0823 # wrist_3_length
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6

        # PLATFORM DISTANCE FROM BASE OF ROBOT
        true_platform_dist = 0.377825 # Based on measurements made in lab
        platform_dist = true_platform_dist - 0.12 # subtract 0.1 to consider
                                    # clearance necessary for radius
                                    # of arm, and also want to grab
                                    # bottle 0.1 meters off ground for COM
        self.platform_dist = platform_dist

        # CUP DISTANCE FROM BASE OF ROBOT
        cup_dist = platform_dist - 0.15 # subtract 0.3 to consider
                                    # clearance necessary for pouring
                                    # into cup below
        self.cup_dist = cup_dist

        # CLOSEST INTERIOR OBSTACLE RADIALLY WITHIN TASKSPACE
        # LOWER BOUND
        int_obstacle_lower = 0.2 # accomodates radius of robot arm and metal pedestal
        # PERP BOUND
        int_obstacle_perp = 0.315045 # accomodates radius of robot arm and metal pedestal
        # UPPER BOUND
        int_obstacle_upper = 0.437861 # accomodates radius of robot arm and metal pedestal
        self.int_obstacle_lower = int_obstacle_lower
        self.int_obstacle_perp = int_obstacle_perp
        self.int_obstacle_upper = int_obstacle_upper

        # PLATFORM LOC - indicates location of platform in space
        platform_loc = (5 * pi) / 4 #UR5e setup in lab
        # platform_loc = pi / 2 # x positive
        # platform_loc = -pi / 2 # x negative
        # platform_loc = pi # y positive
        # platform_loc = 0 # y negative
        self.platform_loc = platform_loc

        # INIT SHOULDER ANGLE
        init_shoulder_angle = 70# in degrees
        self.init_shoulder_angle = init_shoulder_angle

        # EE TO GRIPPER CENTER DISTANCES
        ee_to_gripper = 0.1349375
        ee_to_origin = 0.026824
        self.ee_to_gripper = ee_to_gripper
        self.ee_to_origin = ee_to_origin

        # Initial Variable - indicates picking up or pouring stage
        initial = True
        self.initial = initial

        # Gripper to Cup - distance on x-y plane
        gripper_to_cup = 0.02 # based own experiments pouring bottle into cup
        self.gripper_to_cup = gripper_to_cup
    pass

    def law_of_cosines(self, s1, s2, s3):
        '''
        Solve triangle interior angles using three sides.
        REFERENCED FROM THE FOLLOWING STACK OVERFLOW
        https://stackoverflow.com/questions/14410484/solve-triangle-using-cosine-in-python
        '''
        return math.acos((s3**2 - s2**2 - s1**2)/(-2.0 * s1 * s2))
    pass

    def compute_future_joint_state(self, x, y, radial_distance):
        '''
        Compute the joint state it need to be in to pick-up/pour.
        Returns joint values shoulder_joint, elbow_joint, wrist_1_joint.
        '''
        if self.initial: # Z-axis displacement to reach bottle
            side1 = self.d1 + self.platform_dist - self.d4
            side2 = self.a2
            side3 = self.a3
            side4 = radial_distance
        else: # Z-axis displacement to pour into cup
            # change in geometry as reach outward toward maximum
            # radial range and as z increases to accomodate for cup's height
            side1 = self.d1 + self.cup_dist - self.d4
            side2 = self.a2
            side3 = self.a3
            side4 = radial_distance + self.gripper_to_cup

        bisecting_side = math.sqrt(math.pow((side1), 2) +\
        math.pow((side4), 2))

        theta1 = math.atan(side4 / side1)
        theta5 = (pi / 2) - theta1

        # USE LAW OF COSINES TO FIND INTERIOR ANGLES OF OTHER TRIANGLE
        theta3 = self.law_of_cosines(side2, side3, bisecting_side)
        theta4 = self.law_of_cosines(side3, bisecting_side, side2)
        theta2 = self.law_of_cosines(bisecting_side, side2, side3)

        # Pouring into cup case, geometry of quadrilateral flips
        # mirror image horizontally
        if not(self.initial):
            if radial_distance > 0.41:
                shoulder_joint = -(theta4 + theta5)
                wrist_1_joint = -(theta1 + theta2 - (pi / 2))
            else:
                shoulder_joint = -(theta1 + theta2 - (pi / 2))
                wrist_1_joint = - (theta4 + theta5)
        else:
            shoulder_joint = -(theta1 + theta2 - (pi / 2))
            wrist_1_joint = - (theta4 + theta5)
            self.initial = False
        elbow_joint = pi - theta3
        
        return shoulder_joint, elbow_joint, wrist_1_joint
    pass

    def go_to_radial_joint_state(self, x, y, radial_dist):
        '''
        Calculate joints 2/3/4 angles necessary for radial translation, 
        and move.
        '''
        move_group = self.move_group

        ##Joint Goals:
        ##Joint 0 - Shoulder pan
        ##Joint 1 - Shoulder lift
        ##Joint 2 - Elbow joint
        ##Joint 3 - Wrist joint 1
        ##Joint 4 - Wrist joint 2
        ##Joint 5 - Wrist joint 3

        shoulder, elbow, wrist = self.compute_future_joint_state(x, y, radial_dist)

        # Minimum task space orientation
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.platform_loc
        joint_goal[1] = shoulder # (-tau * 1.0546) / 360
        joint_goal[2] = elbow # (tau * 108.217545) / 360
        joint_goal[3] = wrist # (-tau * 17.16745) / 360
        # joint 4 doesn't change
        # joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    pass

    def init_rotational_position_turn(self, gripper):
        '''
        Turn the gripper away from the wrist and stay clear from during the movement
        '''
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.platform_loc
        if self.initial: # orient for preliminary setup, helps to avoid singularities
            joint_goal[1] = (-tau * self.init_shoulder_angle) / 360
            joint_goal[2] = (tau * 85.96) / 360
            joint_goal[3] = (-tau * 17.522) / 360
            joint_goal[4] = pi
            joint_goal[5] = 0 # will be ultimate orientation 
        

        gripper.OpenGripper()

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        print("INIT POSE: " + str(move_group.get_current_pose()))
        current_joints = move_group.get_current_joint_values()
        print("INIT JOINT VALUES: " + str(current_joints))
        return all_close(joint_goal, current_joints, 0.01)
    pass

    def init_rotational_position(self):
        '''
        Move to non-interfering, basic pose relative to platform location
        '''
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.platform_loc
        if self.initial: # orient for preliminary setup, helps to avoid singularities
            joint_goal[1] = (-tau * self.init_shoulder_angle) / 360
            joint_goal[2] = (tau * 85.96) / 360
            joint_goal[3] = (-tau * 17.522) / 360
            joint_goal[4] = 0
            joint_goal[5] = 0 # will be ultimate orientation 
        

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    pass

    def compute_shoulder_rotation(self, x, y):
        '''
        Comupte the shoulder joint value it need to be at.
        Return future shoulder joint angle
        '''

        move_group = self.move_group

        # Obtain future angle relative to x-pos, unit circle (counterclockwise)
        joint_goal = move_group.get_current_joint_values()

        # workspace is ((5*pi) / 4) < angle < (pi / 4)

        if x == 0: # along y-axis
            angle = self.platform_loc - pi / 2
        else:
            angle = math.atan(y / x)
            if (self.platform_loc == 0) or (self.platform_loc == pi):
            # case checking for different locations
                if x < 0:
                    if self.platform_loc == 0:
                        angle -= pi
                    else:
                        angle += pi
            elif self.platform_loc == (pi / 2):
                angle = angle
            elif self.platform_loc == (-pi / 2):
                angle = angle - pi
            else: # physical system's location, ((5*pi) / 4)
                if x < 0:
                    angle += pi

        # Return future shoulder joint angle
        return angle
    pass

    def move_to_target(self, x, y, shoulder):
        '''
        Calculate joint 1 angle necessary to move clockwise to target, 
        and move.
        '''

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = shoulder

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        print("EE at target: " + str(move_group.get_current_pose()))
        current_joints = move_group.get_current_joint_values()
        print("EE pose: " + str(current_joints))
        return all_close(joint_goal, current_joints, 0.01)
    pass

    def lift_bottle(self):
        '''
        lift bottle, rotate upward and accomodate bottle orientation at EE
        '''

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        curr_joint_angle = joint_goal[1]
        new_angle = (-tau * self.init_shoulder_angle) / 360
        angle_difference = new_angle - curr_joint_angle # returns negative

        # - joint 2 rotation = + ee rotation
        joint_goal[1] = new_angle
        joint_goal[3] -= angle_difference # double negative, add angle
        

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    pass

    def find_shoulder_angle_change(self, x, y, shoulder):
        '''
        Find the shoulder joint angle it need to change to pick-up/puor the bottle
        '''

        offset_side = self.ee_to_gripper - self.ee_to_origin
        radial_distance = math.sqrt(math.pow((x), 2) +\
        math.pow((y), 2))
        bisecting_side = math.sqrt(math.pow((radial_distance), 2) -\
        math.pow(offset_side, 2))
        theta1 = math.asin(offset_side / radial_distance)

        new_shoulder = shoulder + abs(theta1)

        return new_shoulder, bisecting_side
    pass

    def pour_bottle(self):
        '''
        pour bottle, by rotating wrist, sleeps for 5 seconds
        '''

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[3] -= (tau * 105) / 360

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        if not(all_close(joint_goal, current_joints, 0.01)):
            return False
        
        sleep(3) #wait until complete "pour" into cup
        
        joint_goal = move_group.get_current_joint_values()
        joint_goal[3] += (tau * 105) / 360

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        print("MAX JOINTS VALUES: " + str(current_joints))
        print("MAX POSE: "+ str(move_group.get_current_pose()))
        return all_close(joint_goal, current_joints, 0.01)
    pass

    def perform_first_phase(self, x_bottle, y_bottle, gripper):
        '''
        Going from unknown state, move to init pos.
        Move to bottle with open gripper
        '''

        # # Move to non-interfering, basic pose relative to platform location
        self.init_rotational_position()
        # # Turn the gripper away from the wrist and stay clear from during the movement
        self.init_rotational_position_turn(gripper)

        init_shoulder = self.compute_shoulder_rotation(x_bottle, y_bottle)
        shoulder, radial_dist = self.find_shoulder_angle_change(x_bottle, y_bottle, init_shoulder)

        # # calculate joints 2/3/4 angles necessary for radial translation, move
        self.go_to_radial_joint_state(x_bottle, y_bottle, radial_dist)

        # # calculate joint 1 angle necessary to move clockwise to target, move
        self.move_to_target(x_bottle, y_bottle, shoulder)

        # # grab the bottle
        gripper.CloseGripper()
    pass

    def perform_second_phase(self):
        '''
        Once grip bottle, lift and return to init pos.
        '''

        # lift bottle, rotate upward and accomodate bottle orientation at EE
        self.lift_bottle()

        # return to initial start position
        self.init_rotational_position()
    pass

    def perform_third_phase(self, x_cup, y_cup):
        '''
        After receive x,y coords of glass, move to target,
        sleep for pouring, return to init state
        '''

        init_shoulder2 = self.compute_shoulder_rotation(x_cup, y_cup)
        shoulder2, radial_dist2 = self.find_shoulder_angle_change(x_cup, y_cup, init_shoulder2)

        # calculate joints 2/3/4 angles necessary for radial translation, move
        self.go_to_radial_joint_state(x_cup, y_cup, radial_dist2)

        # calculate joint 1 angle necessary to move clockwise to target, move
        self.move_to_target(x_cup, y_cup, shoulder2)

        # pour bottle, by rotating wrist, sleeps for 5 seconds
        self.pour_bottle()

        # lift bottle, rotate upward and accomodate bottle orientation at EE
        self.lift_bottle()

        # return to initial start position
        self.init_rotational_position()
    pass



'''===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v===v==='''
def main():
    try:
        # INIT
        tutorial = MoveGroupUR5PythonInterface()
        gripper = Gripper()
        gripper.gripper_init()
        gripper.CloseGripper()
        # tutorial.init_rotational_position()

        # SCAN FOR FIRST TARGET
        # For testing, prompt user for x and y coordinates
        # Full system integration, get coords. from CV
        out_of_range = True
        while out_of_range:
            x_bottle = float(input("Enter bottle x coord. "))
            y_bottle = float(input("Enter bottle y coord. "))
            e_dist = math.sqrt(math.pow(x_bottle, 2) + math.pow(y_bottle, 2))
            if e_dist < 0.5453264 and e_dist > 0.2: # Maximum distance, reaching furthest corner of table
                out_of_range = False
            else:
                print("Error: Coordinates out of range. Try again.")

        tutorial.perform_first_phase(x_bottle, y_bottle, gripper)

        # # #USE GRIPPER

        tutorial.perform_second_phase()

        # SCAN FOR SECOND TARGET
        # For testing, prompt user for x and y coordinates
        # Full system integration, get coords. from CV
        out_of_range2 = True
        while out_of_range2:
            x_cup = float(input("Enter cup x coord. "))
            y_cup = float(input("Enter cup y coord. "))
            e_dist2 = math.sqrt(math.pow(x_cup, 2) + math.pow(y_cup, 2))
            # Max. Distance based on 0.2 / 0.3 dimensional ratio
            # causing EE to reach limit before collision with self
            if e_dist2 < 0.5453264 and e_dist2 > 0.2: # Maximum distance, reaching furthest corner of table
                out_of_range2 = False
            else:
                print("Error: Coordinates out of range. Try again.")

        tutorial.perform_third_phase(x_cup, y_cup)

        print("Success")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
pass


if __name__ == "__main__":
    main()
