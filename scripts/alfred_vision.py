#!/usr/bin/env python
from builtins import print

import rospy
import cv2
import time
import glob
import tf
import numpy as np

from functools import partial
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.srv import GetMap

PATH = '/home/alexis/catkin_ws/src/alfred'

TARGET_FRAME = 'map'

CAMERA_MATRIX = np.zeros((3, 3))
DISTORTION_MATRIX = np.zeros((1, 5))

IS_LOCKED = False
GOT_CAMERA_INFO = False

ID = 1
SEQ = 0
MARKER_SIZE = 0.04


def is_locked(msg):
    global IS_LOCKED
    IS_LOCKED = msg.data


def get_stamped_pose(translation, rotation):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = TARGET_FRAME
    pose.header.seq = SEQ
    pose.pose.position = Point(*translation)
    pose.pose.orientation = Quaternion(*rotation)
    return pose


def image_analyser(msg, detector, publisher):
    global IS_LOCKED
    global SEQ

    res = rospy.ServiceProxy('dynamic_map', GetMap)().map.info.resolution

    t = rospy.Time(0)
    listener = tf.TransformListener()
    listener.waitForTransform(TARGET_FRAME, 'camera_link', t, rospy.Duration(1))

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    object_pose = None

    start = time.time()

    (corners, ids, rejected) = detector.detectMarkers(img)

    if ids is not None:
        for i in range(0, len(ids)):
            if ids[i][0] == ID:
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, CAMERA_MATRIX,
                                                                                 DISTORTION_MATRIX)
                coef = 10
                tvec = np.divide(np.divide(tvecs[0, 0], res), coef)
                tvec = np.flip(tvec)
                rvec = tf.transformations.quaternion_from_euler(rvecs[0, 0, 0], rvecs[0, 0, 1], rvecs[0, 0, 2])

                object_pose = get_stamped_pose(tvec, rvec)

                IS_LOCKED = True

                publisher.publish(object_pose)

    end = time.time()
    rospy.loginfo('Computing time : %f', end - start)
    rospy.loginfo(object_pose)


def camera_info_callback(msg):
    global GOT_CAMERA_INFO, CAMERA_MATRIX, DISTORTION_MATRIX
    DISTORTION_MATRIX = np.array(msg.D)

    for i, row in enumerate(CAMERA_MATRIX):
        for j, elmt in enumerate(row):
            CAMERA_MATRIX[i, j] = msg.K[i*3+j]

    GOT_CAMERA_INFO = True


def alfred_vision():
    pub = rospy.Publisher('/alfred/target_position', PoseStamped, queue_size=10)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    rospy.init_node('alfred_vision', anonymous=True)

    sub = rospy.Subscriber('/camera/camera_info', CameraInfo, camera_info_callback)

    image_analyser_partial = partial(image_analyser, detector=detector, publisher=pub)
    rospy.Subscriber("/camera/image", Image, image_analyser_partial)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if GOT_CAMERA_INFO:
            sub.unregister()

        rate.sleep()


if __name__ == '__main__':
    try:
        alfred_vision()
    except rospy.ROSInterruptException:
        pass
