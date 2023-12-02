# This code borrows heavily from
# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    We need this function in order to check if the current pose and desired
    pose are close enough.

    Copied from
    https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#

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


# Main object: contains all the actions we want the robot to do
class MoveRobotInterface(object):
    """MoveRobotInterface"""

    def __init__(self):  # object properties
        super(MoveRobotInterface, self).__init__()

        # Initialize ROS nodes to enable computation
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_robot_interface", anonymous=True)

        # This object is an interface to a group of joints
        # This interface can be used to plan and execute motions
        # The group is the primary arm joints in the UR5e robot
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group = move_group

    def go_to_joint_state(self):  # Move arm using a set of joint angles
        move_group = self.move_group

        # Move the arm to a better position to avoid singularities
        # Do this by changing joint values
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0           # shoulder_pan_joint
        joint_goal[1] = -tau / 6    # shoulder_lift_joint: -1/6 of a turn
        joint_goal[2] = tau / 8     # elbow_joint: 1/8 of a turn
        joint_goal[3] = -tau / 8    # wrist_1_joint: -1/8 of a turn
        joint_goal[4] = 0           # wrist_2_joint
        joint_goal[5] = tau / 2     # wrist_3_joint: 1/2 of a turn

        move_group.go(joint_goal, wait=True)  # Move the arm
        move_group.stop()  # Avoid residual movement

        # Print joint angles for testing
        current_joints = move_group.get_current_joint_values()
        print("current_joints")
        print(current_joints)

        # Print position and orientation for testing
        current_pose = self.move_group.get_current_pose().pose
        print("current_pose")
        print(current_pose)

        return all_close(joint_goal, current_joints, 0.01)

    # Plan end effector's path using waypoints to go through
    def plan_cartesian_path(self, scale=1, letter="J"):
        move_group = self.move_group

        waypoints = []

        # Write first initial J: `position.y` does not change, so the end
        # effector stays on the x-z plane
        if letter == "J":
            wpose = move_group.get_current_pose().pose
            wpose.position.z -= scale * 0.3  # Move down (z) 0.3 unit
            waypoints.append(copy.deepcopy(wpose))

            # Moving along two axes simultaneously makes the robot to move
            # diagonally
            wpose.position.z -= scale * 0.15  # Move down (z) 0.15 unit
            wpose.position.x -= scale * 0.15  # Move backwards (x) 0.15 unit
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z += scale * 0.15  # Move up (z) 0.15 unit
            wpose.position.x -= scale * 0.15  # Move backwards (x) 0.15 unit
            waypoints.append(copy.deepcopy(wpose))

        # Write first initial G: `position.y` does not change, so the end
        # effector stays on the x-z plane 
        elif letter == "G":
            wpose = move_group.get_current_pose().pose
            wpose.position.x -= scale * 0.2  # Move backwards (x)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z -= scale * 0.2  # Move down (z)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x += scale * 0.2  # Move forward (x)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z += scale * 0.1  # Move up (z)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x -= scale * 0.1  # Move backward (x)
            waypoints.append(copy.deepcopy(wpose))

        # Write first initial T: `position.y` does not change, so the end
        # effector stays on the x-z plane
        elif letter == "T":
            wpose = move_group.get_current_pose().pose
            wpose.position.x -= scale * 0.3  # Move backward (x)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x += scale * 0.15  # Move forward (x)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z -= scale * 0.3  # Move down (z)
            waypoints.append(copy.deepcopy(wpose))

        # Compute the path or plan
        # Interpolate the path at a 1 cm resolution: 0.01 eef_step
        # No jump threshold: 0.0
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 
        )

        # Return plan but do not move robot yet
        return plan, fraction

    # Move the robot according to a plan that has already been computed
    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

        # Print joint angles for testing
        current_joints = move_group.get_current_joint_values()
        print("current_joints")
        print(current_joints)

        # Print position and orientation for testing
        current_pose = self.move_group.get_current_pose().pose
        print("current_pose")
        print(current_pose)


def main():
    try:
        arm = MoveRobotInterface()  # Main object to interact with

        # Go to a good initial pose so that the robot avoids singularities
        print("Going to initial pose...")
        arm.go_to_joint_state()

        # Wait a few seconds to ensure the robot really goes to the initial
        # position
        time.sleep(3)
        arm.go_to_joint_state()

        # Repeat this process for each of my initials
        for my_initial in "JGT":
            print(f"Writing my initial: {my_initial}...")

            # Plan Cartesian path
            cartesian_plan, fraction = arm.plan_cartesian_path(letter=my_initial)

            # Execute Cartesian path, i.e., write the initial in the air
            arm.execute_plan(cartesian_plan)

            # Returning to initial pose makes it easier to write the next letter
            print("Returning to initial pose...")
            arm.go_to_joint_state()

        print("============ Python demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":  # Run main() when file is called with `python3`
    main()
