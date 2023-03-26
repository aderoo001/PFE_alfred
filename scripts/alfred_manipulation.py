#!/usr/bin/env python
import math

import rospy
import sys
import tf
import moveit_commander

from functools import partial

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp

PARENT_FRAME = 'base_link'
TARGET_FRAME = 'grip_frame'

IS_CATCHABLE = False
IS_CATCH = False
TIMEOUT = 10

OPEN = 0.019
CLOSE = -0.009


def open_gripper(gripper_group):
    gripper_group_variable_values = gripper_group.get_current_joint_values()

    gripper_group_variable_values[0] = OPEN
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    success = gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)
    return success


def close_gripper(gripper_group):
    gripper_group_variable_values = gripper_group.get_current_joint_values()

    gripper_group_variable_values[0] = CLOSE
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    success = gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    return success


def go_home(move_group):
    move_group_variable_values = [0, -1, 0.3, 0.7]
    move_group.set_joint_value_target(move_group_variable_values)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(1)


def move_to_pose(msg, listener, move_group, gripper_group, scene):
    global IS_CATCHABLE, IS_CATCH
    IS_CATCHABLE = msg.data
    print("lul")

    if IS_CATCHABLE and not IS_CATCH:
        print("lul2")
        try:
            (trans, rot) = listener.lookupTransform(PARENT_FRAME, TARGET_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = trans[0]
        target.pose.position.y = trans[1]
        target.pose.position.z = trans[2]
        target.pose.orientation.x = rot[0]
        target.pose.orientation.y = rot[1]
        target.pose.orientation.z = rot[2]
        target.pose.orientation.w = rot[3]
        target_name = "target"
        scene.add_box(target_name, target, size=(0.05, 0.05, 0.05))

        while not open_gripper(gripper_group):
            rospy.sleep(1)

        target_name = "target"

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = trans[0]
        pose_goal.pose.position.y = trans[1]
        pose_goal.pose.position.z = trans[2]
        pose_goal.pose.orientation.w = 1
        move_group.set_pose_target(pose_goal)

        success = False

        while not success:
            success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

        rospy.sleep(5)

        scene.remove_world_object(target_name)

        while not close_gripper(gripper_group):
            rospy.sleep(1)
        IS_CATCH = True

        rospy.sleep(5)

        go_home(move_group)

        IS_CATCHABLE = False


def alfred_manipulation():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('alfred_manipulation')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_goal_orientation_tolerance(math.pi)

    group_name = "gripper"
    gripper_group = moveit_commander.MoveGroupCommander(group_name)
    listener = tf.TransformListener()
    move_to_pose_partial = partial(move_to_pose, listener=listener, move_group=move_group, gripper_group=gripper_group, scene=scene)
    rospy.Subscriber('/alfred/is_catchable', Bool, move_to_pose_partial)

    rospy.spin()


if __name__ == '__main__':
    try:
        alfred_manipulation()
    except rospy.ROSInterruptException:
        pass
