#! /usr/bin/env python3

from __future__ import print_function

import rospy
import roslib
import sys
import time
import cv2
import numpy as np
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

#from licenseTester import findBlueCar

MODE_DICT = {
    0 : "STOP",
    1 : "OUTER_PLAIN",
    2 : "HILL",
    10: "INITIAL_START"
}

class lane_keeper:
    def __init__(self):
        # Subscribers and Publishers
        self.drive_pub = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.drive)
        self.master_sub = rospy.Subscriber('/drive_enb',Int8,self.change_gear)

        # Handler Objects
        self.bridge = CvBridge()
        self.move = Twist()

        # Variables and flags
        self.PID_K = 7
        self.previous_x = 0
        self.mode = "STOP"

    def change_gear(self,msg):
        self.mode = MODE_DICT[msg.data]
        print("Gear changed to:", self.mode)
    
    def drive(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if self.mode == "STOP":
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.drive_pub.publish(self.move)
            return
        elif self.mode == "INITIAL_START":
            self.initial_orient()
            return
        elif self.mode == "OUTER_PLAIN":
            self.outer_plain(cv_image)
        elif self.mode == "HILL":
            self.hill(cv_image)

        

    def initial_orient(self):
        self.move.linear.x = 0.2
        self.drive_pub.publish(self.move)
        time.sleep(2)
        self.move.linear.x = 0
        self.move.angular.z = 1
        self.drive_pub.publish(self.move)
        time.sleep(1.8)
        self.mode = "STOP"

    def outer_plain(self,cv_image):
        # process image using gray-scale filtering
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        # get x value of right lane
        trans = []
        for col in range(frame_bin.shape[1]-1,0,-1):
            if frame_bin[20,col-1] != frame_bin[20,col]:
                trans.append(col)
            if len(trans) == 2:
                break
        if len(trans) == 2 and trans[0] > 440 and trans[1] > 440:
            x = int(round(0.5*sum(trans)))
            self.previous_x = x
        else:
            x = self.previous_x

        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        self.move.linear.x = 0.3 * ((1-0.2)*(x-440)/(1035-440) + 0.2)
        self.move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
        self.drive_pub.publish(self.move)

    def hill(self,cv_image):
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        x = self.previous_x
        for row in [20,0,39]:
            trans = []
            for col in range(frame_bin.shape[1]-1,0,-1):
                if frame_bin[row,col-1] != frame_bin[row,col]:
                    trans.append(col)
                if len(trans) == 2:
                    break
            if len(trans) == 2 and trans[0] > 440 and trans[1] > 440:
                x = int(round(0.5*sum(trans)))
                self.previous_x = x
                break

        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        self.move.linear.x = 0.2 * ((1-0.1)*(x-440)/(1035-440) + 0.1)
        self.move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
        self.drive_pub.publish(self.move)
    
def main(args):
    # Initiate node
    rospy.init_node('lane_keeper', anonymous=True)
    # Create a converter object with Subscribing and Publishing functions
    ic = lane_keeper()
    try:
        # Subscribe to incoming data from camera continously 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down Lane Keeping")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

