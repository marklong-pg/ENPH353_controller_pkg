#! /usr/bin/env python3

from __future__ import print_function
from decimal import DivisionByZero

import rospy
import roslib
#roslib.load_manifest('controller_pkg')
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

#from licenseTester import findBlueCar

class lane_keeper:
    def __init__(self):
        self.drive_pub = rospy.Publisher('R1/cmd_vel',Twist,queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        self.PID_K = 7
        self.previous_x = 0
        self.off_road_count = 0
        self.plate_count = 0
        self.off_road = False
        rospy.spin()
        move = Twist()
        move.linear.x = 0
        move.angular.z = 0
        self.drive_pub.publish(move)
        cv2.destroyAllWindows()

    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # process image using gray-scale filtering
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (H,W) = frame_gray.shape
        frame_blur = cv2.GaussianBlur(frame_gray,(15,15),10)
        frame_cut = frame_blur[int(3*H/4):H,0:W]
        # _, frame_bin = cv2.threshold(frame_cut.astype(np.uint8), 200, 255, cv2.THRESH_BINARY)
        _, frame_bin = cv2.threshold(frame_cut, np.max(frame_cut)-20, 255, cv2.THRESH_BINARY)

        # get x value of lane center
        trans = []
        for col in range(frame_bin.shape[1]-1,0,-1):
            if frame_bin[100,col-1] != frame_bin[100,col]:
                trans.append(col)
            if len(trans) == 2:
                break
        if len(trans) == 2 and trans[0] > 440 and trans[1] > 440:
            x = int(round(0.5*sum(trans)))
            self.previous_x = x
        else:
            x = self.previous_x

        move = Twist()
        move.linear.x = 0.2
        move.angular.z = self.PID_K*(x-1035)/(440 - 1035)
        cv2.imshow("lane_keep",cv2.circle(cv_image.copy(),(x,H-100),20,(0,0,255),-1))
        # cv2.imshow("lane_keep",cv2.circle(frame_bin.copy(),(x,100),20,(0,0,255),-1))
        cv2.waitKey(3)

        # trans = []
        # for col in range(frame_bin.shape[1]-1):
        #     if frame_bin[100,col+1] != frame_bin[100,col]:
        #         trans.append(col+1)
        # if len(trans) == 4:
        #     x = 0.5*(trans[1] + trans[2])
        # elif len(trans) == 3:
        #     if (trans[1] - trans[0]) > (trans[2] - trans[1]):
        #         x = 0.5*(trans[0] + trans[1])
        #     else:
        #         x = 0.5*(trans[2] + trans[1])
        # else:
        #     x = -1
        #     self.off_road_count += 1

        # if x == -1:
        #     x = self.previous_x
        # elif self.off_road_count:
        #     self.off_road_count = 0
        #     self.plate_count += 1
            
        # self.previous_x = x
        # x = int(round(x))
        # print(self.off_road_count)

        # #   Create velocity control message object and publish to topic /cmd_vel
        # move = Twist()
        # if (self.off_road_count > 5 and self.plate_count in [1,2]):
        #     move.linear.x = 0
        #     move.angular.z = 1
        # elif (self.plate_count == 5):
        #     move.linear.x = 0
        #     move.angular.z = 0
        #     print(x)
        # else:
        #     move.linear.x = 0.2
        #     move.angular.z = self.PID_K*(1-x/(W/2))      
        #     # to steer proportional to deviation from line's center
        # print(f"State {self.plate_count}")

        self.drive_pub.publish(move)
    
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

