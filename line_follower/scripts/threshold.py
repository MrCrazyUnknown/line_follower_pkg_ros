#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError


class Threshold:
    def __init__(self):
        rospy.init_node("Threshold", anonymous=True)
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

    def nothing(self, data):
        return None

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv_image = image[image.shape[0]*3/4:image.shape[0], image.shape[1]/4:image.shape[1]*3/4]
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, np.array([0, 0, 0]), np.array([0, 0, 65]))
            mask = cv2.dilate(mask, np.ones((3,3), dtype=np.uint8), iterations=5)
            img, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=lambda x:cv2.contourArea(x))
            if len(contours) > 0:
                x, y, w, h = cv2.boundingRect(contours[-1])
                cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,0,255), 3)
                cv2.line(cv_image, (x + (w/2), 0), (x + (w/2), 360), (255, 0, 0), 3)
                error = ((image.shape[1]/4) - (x + (w/2)))/10
            else:
                error = (image.shape[1]/2)/10
            self.pub.publish(error)
            cv2.imshow('Main', image)
            print(error)
        except CvBridgeError as e:
            print(e)
        if(cv2.waitKey(1) == 27):
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()


if __name__ == '__main__':
    obj = Threshold()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
