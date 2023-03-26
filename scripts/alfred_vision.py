#!/usr/bin/env python
import rospy
import cv2
import time
import tf
import numpy as np

from functools import partial
from cv_bridge import CvBridge

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion

PATH = '/home/alexis/catkin_ws/src/alfred'

TARGET_FRAME = 'map'

CAMERA_MATRIX = np.zeros((3, 3))
DISTORTION_MATRIX = np.zeros((1, 5))

IS_ALLOWED = False
GOT_CAMERA_INFO = False

ID = 1
SEQ = 0
MARKER_SIZE = 0.04


def get_stamped_pose(translation, rotation):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = TARGET_FRAME
    pose.header.seq = SEQ
    pose.pose.position = Point(*translation)
    pose.pose.orientation = Quaternion(*rotation)
    return pose


def image_analyser(msg, detector, publisher):
    global SEQ

    if IS_ALLOWED:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        object_pose = None

        start = time.time()

        (corners, ids, rejected) = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(0, len(ids)):
                if ids[i][0] == ID:
                    rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, CAMERA_MATRIX,
                                                                                     DISTORTION_MATRIX)

                    tvec = tvecs[0, 0]
                    tvec = [tvec[2], -tvec[0], -tvec[1]]
                    rvec = tf.transformations.quaternion_from_euler(rvecs[0, 0, 0], rvecs[0, 0, 1], rvecs[0, 0, 2])
                    rvec[2] = -rvec[2]

                    object_pose = get_stamped_pose(tvec, rvec)

                    publisher.publish(object_pose)

        end = time.time()
        rospy.loginfo('Computing time : %f', end - start)
        rospy.loginfo(object_pose)


def is_allowed(msg):
    global IS_ALLOWED
    IS_ALLOWED = msg.data


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
    rospy.Subscriber('/alfred/allow_vision', Bool, is_allowed)

    image_analyser_partial = partial(image_analyser, detector=detector, publisher=pub)
    rospy.Subscriber("/camera/image", Image, image_analyser_partial)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if GOT_CAMERA_INFO:
            sub.unregister()

        rate.sleep()


if __name__ == '__main__':
    try:
        alfred_vision()
    except rospy.ROSInterruptException:
        pass
