#! /usr/bin/env python3

from __future__ import print_function

import rospy
import roslib
#roslib.load_manifest('controller_pkg')
import sys
import time
import cv2
import numpy as np
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

#from licenseTester import findBlueCar

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
        self.gear = 0

    def change_gear(self,msg):
        self.gear = msg.data
        print(f"Gear changed to: {msg.data}")

    def initial_orient(self):
        self.move.linear.x = 0.2
        self.drive_pub.publish(self.move)
        time.sleep(2)
        self.move.linear.x = 0
        self.move.angular.z = 1
        self.drive_pub.publish(self.move)
        time.sleep(1.8)
        self.gear = 0
    
    def drive(self,data):

        if self.gear == 0:
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.drive_pub.publish(self.move)
            return
        elif self.gear == 10:
            self.initial_orient()
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # process image using gray-scale filtering
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(15,15),10)
        # frame_cut = frame_blur[int(3*H/4):H,0:W]
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        # _, frame_bin = cv2.threshold(frame_cut.astype(np.uint8), 200, 255, cv2.THRESH_BINARY)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        # get x value of lane center
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

        self.move.linear.x = 0.2 #TODO: change linear speed according to self.gear
        self.move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
        # cv2.imshow("lane_keep",cv2.circle(cv_image.copy(),(x,H-100),20,(0,0,255),-1))
        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        self.drive_pub.publish(self.move)
    
def main(args):
    # Initiate node
    rospy.init_node('lane_keeper', anonymous=True)
    # Create a converter object with Subscribing and Publishing functions
    ic = lane_keeper()
    # try:
    #     # Subscribe to incoming data from camera continously 
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting Down Lane Keeping")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

