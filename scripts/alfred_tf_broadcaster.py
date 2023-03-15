#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseStamped

IS_FOUND = False
CAMERA_FRAME = 'camera_link'
PARENT_FRAME = 'map'
TARGET_FRAME = 'target_link'
GOAL_FRAME = 'goal_link'
GRIP_FRAME = 'grip_link'


def send_transform(trans, rot, parent):
    br = tf.TransformBroadcaster()

    br.sendTransform(trans, rot, rospy.Time.now(), TARGET_FRAME, parent)
    br.sendTransform((-0.15, 0.02, 0.00), (0, 0, 0, 1), rospy.Time.now(), GOAL_FRAME, TARGET_FRAME)
    br.sendTransform((0.02, 0.02, 0.00), (0, 0, 0, 1), rospy.Time.now(), GRIP_FRAME, TARGET_FRAME)


def handle_target_pose(msg):
    trans = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    rot = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
           msg.pose.orientation.w)

    send_transform(trans, rot, CAMERA_FRAME)


def loopback(listener):
    try:
        (trans, rot) = listener.lookupTransform(PARENT_FRAME, TARGET_FRAME, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    send_transform(trans, rot, PARENT_FRAME)


def alfred_tf_broadcaster():
    global IS_FOUND

    rospy.init_node('alfred_tf_broadcaster')
    listener = tf.TransformListener()

    sub = rospy.Subscriber('/alfred/target_position', PoseStamped, handle_target_pose)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if listener.canTransform(PARENT_FRAME, TARGET_FRAME, rospy.Time()):
            IS_FOUND = True

        if IS_FOUND:
            sub.unregister()
            loopback(listener)

        rate.sleep()


if __name__ == '__main__':
    try:
        alfred_tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
