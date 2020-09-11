#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CamRcv:
    def __init__(self):
        rospy.init_node("CamRcv", anonymous=True)
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.imshow('Image', cv_image)
        except CvBridgeError as e:
            print(e)

        k = cv2.waitKey(1)

        if(k == 27):
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()



if __name__ == '__main__':
    obj = CamRcv()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
