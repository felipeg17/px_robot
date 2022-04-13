"""
Script intended to use moveit in python for a px_robot
"""
# Python 2/3 compatibility imports
from __future__ import print_function

# General imports
import sys
import copy
import time
import rospy

# ROS and Moveit
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from std_msgs.msg import String

# Transf  and RTB
from tf.transformations import *
from spatialmath import *
from spatialmath.base import *
import roboticstoolbox as rtb

# Tools
import numpy as np

__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"
# Source: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html


# Set clean prints
np.set_printoptions(suppress=True)

#############################################################################
# Starts the moveit_commander node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python", anonymous=False)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Sets the movegroup
# The movegroup is the moveit object for planning, it constains the mechanism, joitns, etc.
# The commander allows the move_group motion
group_name = "px_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
# Planner params
move_group.set_planner_id("PRMkConfigDefault") # Planner PRM, try RRT e.g.
move_group.set_planning_time(5)

#############################################################################
# Joint motion 
# Get the current joints
joint_goal = move_group.get_current_joint_values()
# Set new values
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
# Start the motion
move_group.go(joint_goal, wait=True)
# Finish the motion
move_group.stop()

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -np.pi/6
joint_goal[2] = -np.pi/2
joint_goal[3] = -1*np.pi/3
move_group.go(joint_goal, wait=True)
move_group.stop()
time.sleep(0.1)

# Shows the joints values and the pose
print(move_group.get_current_joint_values())
print(move_group.get_current_pose().pose)
#############################################################################

#############################################################################
# WS Motion
# Get the current pose
current_pose = moveit_commander.conversions.pose_to_list(move_group.get_current_pose().pose)
# Conversion to a ROS msg
pose_goal = geometry_msgs.msg.Pose()
R = trotx(np.pi/2)@troty(np.pi)
qua_goal = quaternion_from_matrix(R)
# New target pose
# NOTE: It may fail, quaterions are super tricky, I just change the Z coordinate
pose_goal.orientation.x = current_pose[3]
pose_goal.orientation.y = current_pose[4]
pose_goal.orientation.z = current_pose[5]
pose_goal.orientation.w = current_pose[6]
pose_goal.position.x = current_pose[0]
pose_goal.position.y = current_pose[1]
pose_goal.position.z = 0.1
# Query the motion
move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
# Stops the motion
move_group.stop()
move_group.clear_pose_targets()

# Shows the joints values and the pose
print(move_group.get_current_joint_values())
print(move_group.get_current_pose().pose)
#############################################################################

#############################################################################
# Cartesian Motion
waypoints = []
wpose = move_group.get_current_pose().pose
scale = 1
wpose.position.z -= scale * 0.05  # First down up (z)
waypoints.append(copy.deepcopy(wpose))
wpose.position.x -= scale * 0.05  # Second move backwards in (x)
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
# Shows the % motion planned, 1 means 100%
print(fraction)
# Execute motion
move_group.execute(plan, wait=True)

# Shows the joints values and the pose
print(move_group.get_current_joint_values())
print(move_group.get_current_pose().pose)
#############################################################################
