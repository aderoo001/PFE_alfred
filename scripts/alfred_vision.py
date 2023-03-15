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
from tf.transformations import translation_matrix, quaternion_matrix, concatenate_matrices

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.srv import GetMap

TARGET_FRAME = 'map'
PATH = '/home/alexis/catkin_ws/src/alfred'
IS_LOCKED = False
ID = 1
SEQ = 0
MARKER_SIZE = 0.04


def calibrate_camera():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    objpoints = []
    imgpoints = []
    images = glob.glob(PATH + '/images/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return mtx, dist


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


def image_analyser(msg, mtx, dist, detector, publisher):
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
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, mtx,
                                                                                 dist)
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


def alfred_vision():
    mtx, dist = calibrate_camera()

    pub = rospy.Publisher('/alfred/target_position', PoseStamped, queue_size=10)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    rospy.init_node('alfred_vision', anonymous=True)

    image_analyser_partial = partial(image_analyser, mtx=mtx, dist=dist, detector=detector, publisher=pub)
    rospy.Subscriber("/camera/image", Image, image_analyser_partial)

    rospy.spin()


if __name__ == '__main__':
    try:
        alfred_vision()
    except rospy.ROSInterruptException:
        pass
