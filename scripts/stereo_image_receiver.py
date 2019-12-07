#!/usr/bin/env python2

import sys
import rospy
from std_msgs.msg import String
import cv2
import PIL
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import pdb

import sensor_msgs.point_cloud2 as pc2

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
        # self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("stereo_cam/left/image_raw", Image, self.monocular_view_callback)
        self.point_sub = rospy.Subscriber(
            name="/rtabmap/odom_local_map",
            data_class=PointCloud2, 
            callback=self.pc2_callback,
            queue_size=10)

    def monocular_view_callback(self, data):        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
    
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

    def pc2_callback(self, data):\
        cloud_points = []
        point_generator = pc2.read_points(data, skip_nans=True)
        while(True):
            try:
                cloud_points.append(point_generator.next())
            except:
                break
        
        rospy.loginfo("Found %s points in cloud" % len(cloud_points))


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