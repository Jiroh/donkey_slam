#!/usr/bin/env python2

import sys
import rospy
from std_msgs.msg import String
import cv2
import PIL
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import pdb

def split_image(img):
    
    img = Image.fromarray(img)
    w,h = img.size
    
    left_area = (0,0,w//2,h)
    right_area = (w//2,0,w,h)
    left_image = img.crop(left_area)
    right_image = img.crop(right_area)
    
    return left_image, right_image


class stereo_image_preprocessor:

    def __init__(self, args=None):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("my_stereo/right/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
    
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)


def main(args):
    sip = stereo_image_preprocessor()
    rospy.init_node("stereo_processor", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)