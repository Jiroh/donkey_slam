#!/usr/bin/env python2

import sys
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from PIL import Image
import pdb
import camera_info_manager
from config import *
from util import *

### STEREO CALIBRATION
## Monocular calibration
# $ rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.108 right:=/stereo_cam/right/image_raw left:=/stereo_cam/left/image_raw right_camera:=/stereo_cam/right left_camera:=/stereo_cam/left
## Stereo calibration (generates topics rtabmap needs, needs to be running constantly)
# $ ROS_NAMESPACE=stereo_cam rosrun stereo_image_proc stereo_image_proc

### RUN RTAB_MAP
# $ roslaunch rtabmap_ros donkey_slam.launch stereo_namespace:="/stereo_cam" rtabmap_args:="--delete_db_on_start"

def main():
    # create ros camera info messages
    l_info_manager = camera_info_manager.CameraInfoManager(
        cname=LEFT_CAM,
        url="file://%s/left.yaml" % CALIBRATION_DATA,
        namespace=LEFT_CAM)
    l_info_manager.loadCameraInfo()
    l_info_message = l_info_manager.getCameraInfo()

    r_info_manager = camera_info_manager.CameraInfoManager(
        cname=RIGHT_CAM,
        url="file://%s/right.yaml" % CALIBRATION_DATA,
        namespace=RIGHT_CAM)
    r_info_manager.loadCameraInfo()
    r_info_message = r_info_manager.getCameraInfo()


    # left camera publishers
    l_img_pub = rospy.Publisher("%s/image_raw" % LEFT_CAM, ImageMsg, queue_size=QUEUE_SIZE)
    l_info_pub = rospy.Publisher("%s/camera_info" % LEFT_CAM, CameraInfo, queue_size=QUEUE_SIZE)

    # right camera publishers
    r_img_pub = rospy.Publisher("%s/image_raw" % RIGHT_CAM, ImageMsg, queue_size=QUEUE_SIZE)
    r_info_pub = rospy.Publisher("%s/camera_info" % RIGHT_CAM, CameraInfo, queue_size=QUEUE_SIZE)

    # initialize node
    rospy.init_node("stereo_image_stream", anonymous=True)

    cap = cv2.VideoCapture(-1)
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
        l_img_message.header.frame_id = HEADER_ID
        r_img_message.header.frame_id = HEADER_ID
        l_info_message.header.frame_id = HEADER_ID
        r_info_message.header.frame_id = HEADER_ID
    
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