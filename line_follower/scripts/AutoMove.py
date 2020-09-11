#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


class AutoMove:
    def __init__(self):
        rospy.init_node('AutoMove', anonymous=True)
        self.sub = rospy.Subscriber('/error', Float32, self.callback)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.error = 0
        self.integral = 0
        self.differential = 0
        self.temp = 0
        cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Kp", "Control Panel", 2, 100, self.nothing)
        cv2.createTrackbar("Ki", "Control Panel", 0, 100, self.nothing)
        cv2.createTrackbar("Kd", "Control Panel", 0, 100, self.nothing)

    def nothing(self, data):
        return None

    def callback(self, data):
        self.error = float(data.data)
        self.integral = self.integral + self.error
        self.ki = cv2.getTrackbarPos("Ki", "Control Panel") * 0.001
        self.kp = cv2.getTrackbarPos("Kp", "Control Panel") * 0.01
        self.kd = cv2.getTrackbarPos("Kd", "Control Panel") * 0.001
        self.differential = self.error - self.temp
        self.temp = self.error
        self.msg.linear.x = 0.2
        self.msg.angular.z = (self.error*self.kp) + (self.integral*self.ki) + (self.differential*self.kd)
        text = "linear x:" + str(round(self.msg.linear.x, 2))
        text += ", y:" + str(round(self.msg.linear.y, 2))
        text += ", z:" + str(round(self.msg.linear.z, 2))
        text += "  angular x:" + str(round(self.msg.angular.x, 2))
        text += ", y:" + str(round(self.msg.angular.y, 2))
        text += ", z:" + str((self.msg.angular.z, 2))
        self.pub.publish(self.msg)
        if cv2.waitKey(1) == 27:
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()


if __name__ == '__main__':
    obj = AutoMove()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
