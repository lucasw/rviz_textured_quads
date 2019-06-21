#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import copy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import tf
import math

# TODO(lucasw) these no longer exist, demo.launch supersedes this test
# is there any reason to keep this around?
# from rviz_textured_quads.msg import TexturedQuad, TexturedQuadArray


def pub_image():

    rospy.init_node('rviz_display_image_test', anonymous=True)
    rospack = rospkg.RosPack()

    image_pub = rospy.Publisher("/targets", Image, queue_size=10)

    texture_path = rospack.get_path('rviz_textured_quads') + '/tests/textures/'
    img1 = cv2.imread(texture_path + 'bebop_drone.jpg', cv2.IMREAD_COLOR)
    img_msg1 = CvBridge().cv2_to_imgmsg(img1, "bgr8")

    img2 = cv2.imread(texture_path + 'Decal.png', cv2.IMREAD_COLOR)
    img_msg2 = CvBridge().cv2_to_imgmsg(img2, "bgr8")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        image_pub.publish(img_msg1)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass
