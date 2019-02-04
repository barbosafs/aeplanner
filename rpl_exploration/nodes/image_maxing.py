#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math

image_pub = None
camera_info_pub = None
camera_info = None
bridge = CvBridge()
max_meters = 7


def imageCallback(msg):
    global camera_info
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        print(e)

    height, width = cv_image.shape[0:2]
    for x in range(0, width):
        for y in range(0, height):
            if math.isnan(cv_image[y, x]):
                cv_image[y, x] = max_meters

    new_msg = bridge.cv2_to_imgmsg(cv_image, "32FC1")
    new_msg.header = msg.header

    image_pub.publish(new_msg)

    camera_info.header.stamp = msg.header.stamp
    camera_info_pub.publish(camera_info)


def image_maxing():
    global image_pub, camera_info_pub, camera_info
    image_pub = rospy.Publisher("/virtual_camera/depth/image_raw", Image, queue_size=100)
    camera_info_pub = rospy.Publisher("/virtual_camera/depth/camera_info", CameraInfo, queue_size=100)
    rospy.init_node("image_maxing", anonymous=True)

    camera_info = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)

    sub = rospy.Subscriber("/camera/depth/image_raw", Image, imageCallback)

    rospy.spin()


if __name__ == '__main__':
    try:
        image_maxing()
    except rospy.ROSInterruptException:
        pass
