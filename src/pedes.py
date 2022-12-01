#! /usr/bin/env python3

from __future__ import print_function


import rospy
import sys
import time
import cv2
import numpy as np
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class pedes_kicker:
    def __init__(self):
        self.drive_pub = rospy.Publisher('/drive_enb',Int8,queue_size=1)
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.pedes_handler)

        # Handler objects
        self.bridge = CvBridge()

        # Variables and flags
        self.at_crosswalk = False
        self.pedes_init_side = -1
        self.LEFT = 0
        self.RIGHT = 1
        self.LEFT_LIM = 400
        self.RIGHT_LIM = 800
        # self.pedes_avoided = False
        self.low_red = np.array([0, 100, 20])
        self.high_red = np.array([10, 255, 255])

        self.low_blue = np.array([100,50,20])
        self.high_blue = np.array([110,140,120])            

    def pedes_handler(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if not self.at_crosswalk:
            self.check_red(cv_image)
        else:
            self.avoid_pedes(cv_image)
    
    def check_red(self, cv_image):
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        (H,W) = hsv.shape[:2]
        hsv = hsv[H-100:H,0:W]
        red_mask = cv2.inRange(hsv,self.low_red,self.high_red)
        red_filtered = cv2.bitwise_and(hsv, hsv, mask=red_mask)

        for j in range(550,650):
            if red_filtered[60,j,2] > 0:
                self.at_crosswalk = True
                self.drive_pub.publish(0)
                print('Crosswalk detected!')
                break

    def avoid_pedes(self, cv_image):
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        W = hsv.shape[1]
        hsv = hsv[330:480,:,:]
        blue_mask = cv2.inRange(hsv,self.low_blue,self.high_blue)
        blue_filtered = cv2.bitwise_and(hsv,hsv,mask=blue_mask)

        x_pedes = -1
        for i in range(75,90):
            detect = False
            for j in range(W):
                if blue_filtered[i,j,0] > 0:
                    x_pedes = j
                    detect = True
                    break
            if detect:
                break
        if x_pedes == -1:
            return

        if self.pedes_init_side == -1:
            self.pedes_init_side = int(x_pedes > 600)
            print(f'Initial Side = {self.pedes_init_side}')
            return
        
        # print(f'Pedes:{x_pedes}')
        
        if  (self.pedes_init_side == self.LEFT and x_pedes > self.RIGHT_LIM) or \
            (self.pedes_init_side == self.RIGHT and x_pedes < self.LEFT_LIM):
            self.drive_pub.publish(1)
            time.sleep(10)
            self.reset()

    def reset(self):
        self.at_crosswalk = False
        self.pedes_init_side = -1
        print('Pedes tracker reset successfully!')

def main(args):
    # Initiate node
    rospy.init_node('pedes_kicker', anonymous=True)
    # Create a converter object with Subscribing and Publishing functions
    ic = pedes_kicker()
    try:
        # Subscribe to incoming data from camera continously 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Pedes Kicker")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
        






        
    