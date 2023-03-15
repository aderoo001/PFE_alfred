#!/usr/bin/env python

import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

IS_FOUND = False
TARGET_FRAME = 'goal_link'
MAP_FRAME = 'map'
SEQ = 0


def alfred_navigation():
    global IS_FOUND
    global SEQ

    rospy.init_node('alfred_navigation')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    listener = tf.TransformListener()

    while not IS_FOUND:
        if listener.canTransform(MAP_FRAME, TARGET_FRAME, rospy.Time()):
            (trans, rot) = listener.lookupTransform(MAP_FRAME, TARGET_FRAME, rospy.Time(0))

            goal = MoveBaseGoal()

            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = MAP_FRAME
            goal.target_pose.header.seq = SEQ

            goal.target_pose.pose.position.x = trans[0]
            goal.target_pose.pose.position.y = trans[1]
            goal.target_pose.pose.position.z = 0

            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = -rot[2]
            goal.target_pose.pose.orientation.w = -rot[3]
            rospy.loginfo(goal)

            client.send_goal(goal)
            client.wait_for_result()

            IS_FOUND = True


if __name__ == '__main__':
    try:
        alfred_navigation()
    except rospy.ROSInterruptException:
        pass
