#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
from line_follower.cfg import PIDControlConfig
from dynamic_reconfigure.server import Server


class Movement:
    def __init__(self):
        rospy.init_node("Movement")
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.msg = Twist()
        self.param = {"KP":0.005, "KI":0, "KD":0, "SP":0.2, "ROI_W":400, "ROI_H": 400, "ROI_Y": 300}
        self.integral = self.prev_error = 0
        srv = Server(PIDControlConfig, self.reconfig)

    def callback(self, data):
        try:
            input_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown("shutdown")

        error = self.compute_error(input_img)
        rectify = self.pid(error)

        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = rectify
        self.pub.publish(self.msg)
        self.rate.sleep()

    def reconfig(self, config, level):
        self.param = config
        return config

    def compute_error(self, img_in):
        roi_y = self.param["ROI_Y"]
        roi_h = self.param["ROI_H"]
        roi_w = self.param["ROI_W"]
        if roi_w > img_in.shape[1]:
            rospy.loginfo("ROI_W surpassed the image bounds")
            roi_w = img_in.shape[1]
        if (roi_y + (roi_h/2)) > img_in.shape[0]:
            rospy.loginfo("ROI_Y surpassed the image bounds")
            roi_y = img_in.shape[0] - (roi_h/2)
        roi = img_in[int(roi_y - (roi_h/2)):int(roi_y + (roi_h/2)), int((img_in.shape[1]/2) - (roi_w/2)):int((img_in.shape[1]/2) + (roi_w/2))]
        hsv_img = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        masked = cv2.inRange(hsv_img, np.array([0, 0, 0]), np.array([0, 0, 65]))
        masked = cv2.dilate(masked, np.ones((3, 3), dtype=np.uint8), iterations=5)
        img, contours, hierarchy = cv2.findContours(masked.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x))
        if len(contours) > 0:
            x, y, w, h = cv2.boundingRect(contours[-1])
            cv2.rectangle(roi, (x,y), (x+w, y+h), (0, 0, 255), 2)
            cv2.line(roi, (x + (w/2), y), (x + (w/2), y+h), (255, 0, 0), 2)
            balance = (roi.shape[1]/2 - (x + (w/2)))
        else:
            if self.prev_error > 0:
                balance = roi.shape[1]/2
            else:
                balance = -1 * roi.shape[1]/2
        cv2.rectangle(img_in, (int((img_in.shape[1]/2) - (roi_w/2)), int(roi_y - (roi_h/2))), (int((img_in.shape[1]/2) + (roi_w/2)), int(roi_y + (roi_h/2))), (0, 255, 100), 2)
        rospy.loginfo(balance)
        cv2.imshow("Main", img_in)
        if cv2.waitKey(1) == 27:
            self.msg.linear.x = 0
            self.msg.angular.z = 0
            self.pub.publish(self.msg)
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()
        return balance

    def pid(self, err):
        self.integral += err
        diff = err - self.prev_error
        self.prev_error = err
        return (self.param["KP"] * err) + (self.param["KI"] * 0.01 * self.integral) + (self.param["KD"] * 0.01 * diff)


if __name__ == '__main__':
    move = Movement()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
