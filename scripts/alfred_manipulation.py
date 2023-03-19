#!/usr/bin/env python

import rospy
import sys
import tf
import moveit_commander

from functools import partial

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

PARENT_FRAME = 'map'
TARGET_FRAME = 'grip_link'


def move_to_pose(msg, listener, move_group):
    if msg.data:
        try:
            (trans, rot) = listener.lookupTransform(PARENT_FRAME, TARGET_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.position.x = trans[0]
        pose_goal.position.y = trans[1]
        pose_goal.position.z = trans[2]

        # quaternion = tf.transformations.quaternion_from_euler(0, 180, 0)
        # pose_goal.orientation.x = 0#quaternion[0]
        # pose_goal.orientation.y = 0#quaternion[1]
        # pose_goal.orientation.z = 0#quaternion[2]
        # pose_goal.orientation.w = 1#quaternion[3]

        rospy.loginfo(pose_goal)

        move_group.set_pose_reference_frame("base_link")
        move_group.set_pose_target(pose_goal)

        plan = move_group.plan()

        move_group.execute(plan)

        # success = move_group.go(wait=True)
        # move_group.stop()
        # move_group.clear_pose_targets()


def alfred_manipulation():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('alfred_navigation')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

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

    listener = tf.TransformListener()

    move_to_pose_partial = partial(move_to_pose, listener=listener, move_group=move_group)
    rospy.Subscriber('/alfred/is_catchable', Bool, move_to_pose_partial)

    rospy.spin()


if __name__ == '__main__':
    try:
        alfred_manipulation()
    except rospy.ROSInterruptException:
        pass
