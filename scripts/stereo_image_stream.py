#!/usr/bin/env python2

import sys
import numpy as np
import cv2
import rospy
import sensor_msgs
from cv_bridge import CvBridge
from PIL import Image
import pdb
import camera_info_manager

### STEREO CALIBRATION
# $ rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.108 right:=/stereo_cam/right/image_raw left:=/stereo_cam/left/image_raw right_camera:=/stereo_cam/right left_camera:=/stereo_cam/left
# 


def split_image(img):
    img = Image.fromarray(img)
    w,h = img.size
    
    left_area = (0,0,w//2,h)
    right_area = (w//2,0,w,h)
    left_image = img.crop(left_area)
    right_image = img.crop(right_area)
    
    return np.array(left_image), np.array(right_image)


def main():

    # create ros camera info messages
    l_info_manager = camera_info_manager.CameraInfoManager(
        cname="/stereo_cam/left",
        url="file:///left__stereo_camera.yml",
        namespace="/stereo_cam/left")
    l_info_manager.loadCameraInfo()
    l_info_message = l_info_manager.getCameraInfo()

    r_info_manager = camera_info_manager.CameraInfoManager(
        cname="/stereo_cam/right",
        url="file:///right_stereo_camera.yml",
        namespace="/stereo_cam/right")
    r_info_manager.loadCameraInfo()
    r_info_message = r_info_manager.getCameraInfo()

    
    # left camera publishers
    l_img_pub = rospy.Publisher("/stereo_cam/left/image_raw", sensor_msgs.msg.Image, queue_size=10)
    l_info_pub = rospy.Publisher("/stereo_cam/left/camera_info", sensor_msgs.msg.CameraInfo, queue_size=10)

    # right camera publishers
    r_img_pub = rospy.Publisher("/stereo_cam/right/image_raw", sensor_msgs.msg.Image, queue_size=10)
    r_info_pub = rospy.Publisher("/stereo_cam/right/camera_info", sensor_msgs.msg.CameraInfo, queue_size=10)

    # initialize node
    rospy.init_node("stereo_image_stream", anonymous=True)

    cap = cv2.VideoCapture(2)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # preprocess stereo images
        _, raw_image = cap.read()
        l_img, r_img = split_image(raw_image)
        rospy.loginfo("Read Cv Image: " + str(raw_image.shape))
        
        # create ros image messages
        l_img_message = bridge.cv2_to_imgmsg(l_img, encoding="bgr8")
        r_img_message = bridge.cv2_to_imgmsg(r_img, encoding="bgr8")

        # set headers
        header_frame_id = "/camera_link"
        l_img_message.header.frame_id = header_frame_id
        r_img_message.header.frame_id = header_frame_id
        l_info_message.header.frame_id = header_frame_id
        r_info_message.header.frame_id = header_frame_id

    
        # publish messages
        l_img_pub.publish(l_img_message)
        r_img_pub.publish(r_img_message)
        l_info_pub.publish(l_info_message)
        r_info_pub.publish(r_info_message)
        
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass