#!/usr/bin/env python2

import sys
import numpy as np
import cv2
import rospy
import sensor_msgs
from cv_bridge import CvBridge
from PIL import Image
import pdb


def split_image(img):
    img = Image.fromarray(img)
    w,h = img.size
    
    left_area = (0,0,w//2,h)
    right_area = (w//2,0,w,h)
    left_image = img.crop(left_area)
    right_image = img.crop(right_area)
    
    return np.array(left_image), np.array(right_image)


def main():
    l_img_pub = rospy.Publisher("/my_stereo/left/image_raw", sensor_msgs.msg.Image, queue_size=10)
    r_img_pub = rospy.Publisher("/my_stereo/right/image_raw", sensor_msgs.msg.Image, queue_size=10)
    rospy.init_node("stereo_image_stream", anonymous=True)
    # rate = rospy.Rate(1) # 10 hz

    cap = cv2.VideoCapture(3)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, raw_image = cap.read()
        l_img, r_img = split_image(raw_image)

        rospy.loginfo("Read Cv Image: " + str(raw_image.shape))

        l_img_message = bridge.cv2_to_imgmsg(l_img)
        r_img_message = bridge.cv2_to_imgmsg(r_img)

        l_img_message.header.frame_id = "/camera_link"
        r_img_message.header.frame_id = "/camera_link"
        l_img_pub.publish(l_img_message)
        r_img_pub.publish(r_img_message)
        # rate.sleep()
        
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass