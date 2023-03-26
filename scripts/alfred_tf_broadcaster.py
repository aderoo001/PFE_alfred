#!/usr/bin/env python

import rospy
import tf

import moveit_commander

from geometry_msgs.msg import PoseStamped

CAMERA_FRAME = 'camera_rgb_frame'
PARENT_FRAME = 'map'
TARGET_FRAME = 'target_link'
GOAL_FRAME = 'goal_frame'
GRIP_FRAME = 'grip_frame'

ROTATION = ()
TRANSLATION = ()
IS_FOUND = False
PAUSE = False


def send_transform(parent):
    br = tf.TransformBroadcaster()

    br.sendTransform(TRANSLATION, ROTATION, rospy.Time.now(), TARGET_FRAME, parent)
    br.sendTransform((-0.19, 0, 0.00), (0, 0, 0, 1), rospy.Time.now(), GOAL_FRAME, TARGET_FRAME)
    br.sendTransform((0.05, 0, 0.01), (0, 0, 0, 1), rospy.Time.now(), GRIP_FRAME, TARGET_FRAME)


def handle_target_pose(msg, listener):
    global TRANSLATION, ROTATION, IS_FOUND, PAUSE
    PAUSE = True

    TRANSLATION = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    ROTATION = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                msg.pose.orientation.w)

    send_transform(CAMERA_FRAME)

    try:
        (TRANSLATION, ROTATION) = listener.lookupTransform(PARENT_FRAME, TARGET_FRAME, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    IS_FOUND = True
    PAUSE = False


def loopback():
    send_transform(PARENT_FRAME)


def alfred_tf_broadcaster():
    global TRANSLATION, ROTATION, IS_FOUND

    rospy.init_node('alfred_tf_broadcaster')

    listener = tf.TransformListener()

    scene = moveit_commander.PlanningSceneInterface()

    sub = rospy.Subscriber('/alfred/target_position', PoseStamped, handle_target_pose, listener)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if IS_FOUND and not PAUSE:
            # sub.unregister()
            loopback()

        rate.sleep()


if __name__ == '__main__':
    try:
        alfred_tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
