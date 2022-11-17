#!/usr/bin/env python3
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time
import cv2
import gym
import math
import rospy
import roslaunch
import time
import numpy as np
# import imutils

from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from time import sleep

from gym.utils import seeding

from matplotlib import pyplot as plt
from PIL import Image as im

#blue thresholds for the hsv image, tuned for vehicle blue
lower_blue = np.array([100, 125, 100])
upper_blue = np.array([255, 255, 255])
contourXDistThresh=400
contourYDistThresh=100
foundCounter=0

def callback(data, args):
    #print("message recieved, callback called") 
    br = CvBridge()

    #rospy.loginfo("receiving video frame")
    frame = br.imgmsg_to_cv2(data,desired_encoding='bgr8')
    height, width, depth = frame.shape

    #use colour filtering to get only vehicles
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result = cv2.bitwise_and(frame, frame, mask = mask)
    #cv2.imshow("masked",result)

    #Find contours to verify how close we are to the vehicle
    img = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours=[contour for contour in contours if cv2.contourArea(contour)]
    carFound=False

    #If we are detecting 2 wedges
    if len(contours)>2:
        sortedContours=sorted(contours, key=cv2.contourArea, reverse=True) 
        centerList=[]
        
        #If both wedges are big enough for us to be close to the vehicle
        if(cv2.contourArea(sortedContours[0])>10000 and cv2.contourArea(sortedContours[1])>3000):
            print("STOP: CAR DETECTED")
            #need to change this so that I can detect the cars based on the 2 largest contours that are less that some distance apart (<400 from inital testing)
            #solution appears to be look at the largest 3 contours, sort based on minimization of center in x
            for i in range(2):
                M = cv2.moments(sortedContours[i])
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centerList.append(center)
            if abs(centerList[1][0]-centerList[0][0])>contourXDistThresh and abs(centerList[1][1]-centerList[0][1])<contourYDistThresh:
                if(len(contours)>3):
                    M = cv2.moments(sortedContours[2])
                    centerThree = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    centerList.append(centerThree)
                    if(abs(centerList[0][0]-centerList[2][0])<contourXDistThresh) and abs(centerList[1][1]-centerList[2][1])<contourYDistThresh:
                        cv2.circle(frame, centerList[0], 10, (0,0,255), -1)
                        cv2.circle(frame, centerList[2], 10, (0,0,255), -1)
                        print("putting g3")
                        carFound=True
            else:
                for center in centerList:
                    #cv2.circle(frame, center, 10, (0,0,255), -1)
                    carFound=True

            
            #save plate logic
            if carFound:
                args[2]+=1
                print("car edge1 center: " + str(centerList[0]))
                print("car edge2 center: " + str(centerList[1]))
                rgbFrame=cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # foundCar=im.fromarray(rgbFrame)
                # foundCar.save("/home/fizzer/ros_ws/src/2022_competition/enph353/enph353_utils/scripts/plateViews/"+"plate"+str(args[2])+".png")
                print("plate"+str(args[2])+".png is saved")
                #time.sleep(2)
        #After I detect the car, I need to zero in on the license plate
        #After this find various homographies for stopping positions, train neural net (blue and transform license "paltes to train)
        #test together
    else:
        print("keep going")
    
    cv2.imshow("car?",frame)
    cv2.waitKey(3)

def findPlate():
    return 0

def recieve_message():
    rospy.init_node('video_sub_py', anonymous=True)
    twistObj=Twist()
    cmd_vel_pub=rospy.Publisher('/R1/cmd_vel',Twist, queue_size=1) 
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback,[twistObj,cmd_vel_pub,foundCounter])
    print("keepgettingcalls")
    rospy.spin()
    cv2.destroyAllWindows()

def findBlueCar(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result = cv2.bitwise_and(frame, frame, mask = mask)
    #cv2.imshow("masked",result)

    #Find contours to verify how close we are to the vehicle
    img = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours=[contour for contour in contours if cv2.contourArea(contour)]
    carFound=False

    #If we are detecting 2 wedges
    if len(contours)>2:
        sortedContours=sorted(contours, key=cv2.contourArea, reverse=True) 
        centerList=[]
        
        #If both wedges are big enough for us to be close to the vehicle
        if(cv2.contourArea(sortedContours[0])>10000 and cv2.contourArea(sortedContours[1])>3000):
            print("STOP: CAR DETECTED")
            #need to change this so that I can detect the cars based on the 2 largest contours that are less that some distance apart (<400 from inital testing)
            #solution appears to be look at the largest 3 contours, sort based on minimization of center in x
            for i in range(2):
                M = cv2.moments(sortedContours[i])
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centerList.append(center)
            if abs(centerList[1][0]-centerList[0][0])>contourXDistThresh and abs(centerList[1][1]-centerList[0][1])<contourYDistThresh:
                if(len(contours)>3):
                    M = cv2.moments(sortedContours[2])
                    centerThree = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    centerList.append(centerThree)
                    if(abs(centerList[0][0]-centerList[2][0])<contourXDistThresh) and abs(centerList[1][1]-centerList[2][1])<contourYDistThresh:
                        cv2.circle(frame, centerList[0], 10, (0,0,255), -1)
                        cv2.circle(frame, centerList[2], 10, (0,0,255), -1)
                        print("putting g3")
                        carFound=True
            else:
                for center in centerList:
                    #cv2.circle(frame, center, 10, (0,0,255), -1)
                    carFound=True
    return carFound

if __name__ == '__main__':
    recieve_message()
    print("keep getting calls")