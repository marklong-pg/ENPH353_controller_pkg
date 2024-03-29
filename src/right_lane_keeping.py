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
    3 : "TRANSITION_LEFT",
    4 : "TARZAN_LEFT",
    6 : "TARZAN_RIGHT",
    7 : "INNER_RIGHT",
    8 : "CHECK_TRUCK",
    9 : "FINAL_PARKING",
    10: "INITIAL_START"
}

class lane_keeper:
    def __init__(self):
        # Subscribers and Publishers
        self.drive_pub = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.drive)
        self.master_sub = rospy.Subscriber('/drive_enb',Int8,self.change_gear)
        self.master_pub = rospy.Publisher('/inner_trigger',Int8,queue_size=1)

        # Handler Objects
        self.bridge = CvBridge()
        self.move = Twist()

        # Variables and flags
        self.PID_K = 7
        self.previous_x = 0
        self.mode = "STOP"
        self.prev_left_stat = True
        self.prev_right_stat = True
        self.right_recover_count = 0
        self.left_miss_count = 0
        self.right_miss_count = 0

        self.see_box_before = False

    def change_gear(self,msg):
        self.mode = MODE_DICT[msg.data]
        print("Mode changed to:", self.mode)
    
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
        elif self.mode == "TRANSITION_LEFT":
            self.inner_left(cv_image)
        elif self.mode == "CHECK_TRUCK":
            self.check_truck(cv_image)
        # elif self.mode == "INNER_LEFT":
        #     self.inner_left(cv_image, left_limit=True)
        elif self.mode == "INNER_RIGHT":
            self.inner_right(cv_image)
        elif self.mode == "TARZAN_RIGHT":
            self.inner_drive_1(cv_image)
        elif self.mode == "TARZAN_LEFT":
            self.inner_drive_2(cv_image)
            # self.inner_left(cv_image)
        elif self.mode == "FINAL_PARKING":
            self.final_park()
            return
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
    
    def final_park(self):
        print("Final park procedure starts...")
        self.move.linear.x = 0.3
        self.move.angular.z = 0
        self.drive_pub.publish(self.move)
        time.sleep(0.5)
        self.move.linear.x = 0
        self.move.angular.z = 1
        self.drive_pub.publish(self.move)
        time.sleep(2)
        self.move.linear.x = 0.2
        self.move.angular.z = 0
        self.drive_pub.publish(self.move)
        time.sleep(1.5)
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
        # self.move.linear.x = 0.2 * ((1-0.2)*(x-440)/(1035-440) + 0.2)
        self.move.linear.x = 0.2
        self.move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
        self.drive_pub.publish(self.move)

    def check_truck(self,cv_image):
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[int(0.6*H):H,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, 20, 255, cv2.THRESH_BINARY_INV)

        for row in range(0,150):
            for col in range(0,W):
                if frame_bin[row,col] != 0:
                    self.mode = "STOP"
                    print("Saw truck! Waiting to pass....")
                    time.sleep(7) # time to wait for truck to go away
                    self.master_pub.publish(0)
                    return
    
    def inner_right(self,cv_image):
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
            return

        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        target = 1035
        self.move.linear.x = 0.3 
        self.move.angular.z = self.PID_K*(x-target)/(440 - target)
        self.drive_pub.publish(self.move)

    def inner_left(self, cv_image, left_limit=False):
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        # get x value of left lane
        trans = []
        for col in range(0,frame_bin.shape[1]-1):
            if frame_bin[20,col+1] != frame_bin[20,col]:
                trans.append(col)
            if len(trans) == 2:
                break
        
        if len(trans) == 2 and trans[0] < 900 and trans[1] < 900:
            x = int(round(0.5*sum(trans)))
            if left_limit and x < 130:
                x = self.previous_x
            else:
                self.previous_x = x
        else:
            x = self.previous_x
        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        
        self.move.linear.x = 0.3 
        self.move.angular.z = -7*(x-220)/(900 - 220)
        self.drive_pub.publish(self.move)

    def inner_drive(self,cv_image):
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        trans = []
        for col in range(frame_bin.shape[1]-1,0,-1):
            if frame_bin[20,col-1] != frame_bin[20,col]:
                trans.append(col)
            if len(trans) == 2:
                break
        if len(trans) == 2 and abs(trans[0] - trans[1]) < 100:
            x = int(round(0.5*sum(trans)))
            self.previous_x = x
        else:
            x = self.previous_x

        self.move.linear.x = 0.3
        if x > 440:
            self.move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
            self.see_box_before = False
        else:
            # self.mode = "STOP"
            # return
            self.move.angular.z = -7*(x-220)/(900 - 220)
            if not self.see_box_before:
                cv2.imshow("inner_box,",cv_image) 
                self.see_box_before = True  
        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        self.drive_pub.publish(self.move)

    def inner_drive_1(self,cv_image):
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        trans = []
        for col in range(frame_bin.shape[1]-1,0,-1):
            if frame_bin[20,col-1] != frame_bin[20,col]:
                trans.append(col)
            # if len(trans) == 2:
            #     break
        if len(trans) >= 2 and abs(trans[0]-trans[1]) < 100:
            x = int(round(0.5*sum(trans)))
            self.previous_x = x
        else:
            x = self.previous_x

        if len(trans) == 2:
            self.left_miss_count += 1
            print(self.left_miss_count)
            if self.left_miss_count >= 15:
                left_stat = False
            else:
                left_stat = True
        else:
            self.left_miss_count = 0
            left_stat = True
            # print("LEFT now TRUE")
        # print(f"Left stat: {left_stat}; Prev stat: {self.prev_left_stat}")
        
        # if len(trans) != 4:
        #     left_stat = False
        # else:
        #     left_stat = True

        self.move.linear.x = 0.3
        if x > 440: # Line at Right
            self.move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
            self.see_box_before = False
            # if self.prev_left_stat == False and left_stat == True:
            #     if self.left_recover_count == 2:      
            #         self.prev_left_stat = True
            #         self.left_miss_count = 0
            #         # self.mode = "TRANSITION_LEFT"
            #         self.mode = "STOP"
            #         print("Saw left again, swinging to left")
            #         return
            #     else:
            #         self.left_recover_count += 1
            #         print(self.left_recover_count)

            if self.prev_left_stat == False and left_stat == True:
                self.left_miss_count = 0
                # self.mode = "TRANSITION_LEFT"
                # self.mode = "STOP"
                self.mode = "TARZAN_LEFT"
                print("Saw left again, swinging to left")

        # else:
        #     self.move.angular.z = -7*(x-220)/(900 - 220)
        #     if not self.see_box_before:
        #         # cv2.imshow("inner_box,",cv_image) 
        #         self.see_box_before = True
        self.prev_left_stat = left_stat

        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        self.drive_pub.publish(self.move)
    
    def inner_drive_2(self,cv_image):
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(21,21),10)
        frame_cut = frame_blur[640-20:640+20,0:W].astype(np.uint8)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-70, 255, cv2.THRESH_BINARY)

        trans = []
        for col in range(0,frame_bin.shape[1]-1):
            if frame_bin[20,col+1] != frame_bin[20,col]:
                trans.append(col)
            # if len(trans) == 2:
            #     break
        if len(trans) >= 2 and trans[0] < 900 and trans[1] < 900:
            x = int(round(0.5*(trans[0]+trans[1])))
            self.previous_x = x
        else:
            x = self.previous_x

        if len(trans) == 2 and x < 500:
            self.right_miss_count += 1
            print(self.right_miss_count)
            if self.right_miss_count >= 10:
                right_stat = False
            else:
                right_stat = True
        else:
            self.right_miss_count = 0
            right_stat = True

        self.move.linear.x = 0.3
        self.move.angular.z = -self.PID_K*(x-220)/(900 - 220)   
        self.drive_pub.publish(self.move)

        if self.prev_right_stat == False and right_stat == True:
            self.right_miss_count = 0
            if self.right_recover_count == 1:
                # self.mode = "TARZAN_RIGHT"
                # self.mode = "STOP"
                print("Saw right again, swinging to right")
                self.master_pub.publish(1)
                self.mode = "STOP"
                return
            else:
                self.right_recover_count += 1
                print(f"right_recover_count increased to {self.right_recover_count}")
        self.prev_right_stat = right_stat

        cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        cv2.waitKey(3)
        

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

        # cv2.imshow("lane_keep,",cv2.circle(cv2.cvtColor(frame_bin, cv2.COLOR_GRAY2BGR),(x,20),20,(0,0,255),-1))
        # cv2.waitKey(3)
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

