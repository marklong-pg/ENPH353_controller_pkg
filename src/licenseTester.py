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
import functools
import time
import numpy as np
# import imutils

from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep

from gym.utils import seeding

from matplotlib import pyplot as plt
from PIL import Image as im
import sys


class plateProcessor:
    def __init__(self):
        foundCounter=0
        #self.founCounterPub=rospy.Publisher("/R1/foundCounter",int,queue_size=1)
        self.prevCarTime=0
        self.foundCounter=0
        self.startTime=time.time()
        self.imList=[]
        self.saveTag=False
        self.plate_pub=rospy.Publisher('/license_plate', String, queue_size=1)
        rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.carDetect,foundCounter,queue_size=1, buff_size=(1000000))
        rospy.spin()
        cv2.destroyAllWindows()

    def captureSimFeed(self,data,args):
        br = CvBridge()
        frame = br.imgmsg_to_cv2(data,desired_encoding='bgr8')
        if(time.time()-self.startTime<45):
            self.imList.append(frame)
        elif not self.saveTag:
            self.saveTag=True
            i=0
            print("started saving")
            for img in self.imList:
                i+=1
                imgToSave=im.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                imgToSave.save("simFrames/frame"+str(i)+".png")
                print("saving"+str(i))

    def carDetect(self, data, args):
        br = CvBridge()
        frame = br.imgmsg_to_cv2(data,desired_encoding='bgr8')
        
        if(time.time()-self.prevCarTime<2):
            print("im shleep")
            return

        #blue thresholds for the hsv image, tuned for vehicle blue
        lower_blue = np.array([100, 125, 100])
        upper_blue = np.array([255, 255, 255])
        contourXDistThresh=250
        contourYDistThresh=100
        tooCloseXThresh=30
        
        print("stay woke")

        #rospy.loginfo("receiving video frame")

        #use colour filtering to get only vehicles
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        result = cv2.bitwise_and(frame, frame, mask = mask)
        frame2=np.copy(frame)
        #cv2.imshow("masked",result)

        #Find contours to verify how close we are to the vehicle
        img = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours=[contour for contour in contours if cv2.contourArea(contour)]
        carFound=False

        #If we are detecting 2 wedges
        if len(contours)>=2:
            sortedContours=sorted(contours, key=cv2.contourArea, reverse=True) 
            centerList=[]
            
            #If both wedges are big enough for us to be close to the vehicle
            if(cv2.contourArea(sortedContours[0])>15000 and cv2.contourArea(sortedContours[1])>2000):
                print("STOP: CAR DETECTED")
                #mage=im.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                #mage.save("carDetected.png")

                #check that the first two contours are close together to confirm they are part of the
                # same car, if not check the first one with the third. 
                for i in range(2):
                    M = cv2.moments(sortedContours[i])
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    centerList.append(center)
                if (abs(centerList[1][0]-centerList[0][0])>contourXDistThresh or
                 abs(centerList[1][0]-centerList[0][0])<tooCloseXThresh) or (abs(centerList[1][1]-centerList[0][1])
                 >contourYDistThresh):
                    if(len(contours)>3):
                        M = cv2.moments(sortedContours[2])
                        centerThree = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        centerList.append(centerThree)
                        if(abs(centerList[0][0]-centerList[2][0])<contourXDistThresh) and abs(centerList[1][1]-centerList[2][1])<contourYDistThresh:
                            cv2.circle(frame2, centerList[0], 10, (0,0,255), -1)
                            cv2.circle(frame2, centerList[2], 10, (0,0,255), -1)
                            print("putting g3")
                            carFound=True
                        centerList=[centerList[0],centerList[2]]
                elif abs(centerList[1][0]-centerList[0][0])<=contourXDistThresh and abs(centerList[1][1]-centerList[0][1])<=contourYDistThresh:
                    for center in centerList:
                        cv2.circle(frame2, center, 10, (0,0,255), -1)
                        carFound=True
                
                #save plate logic
                if carFound:
                    args+=1
                    rgbFrame=cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                    zeroedInEdges=self.locateCar(centerList,frame.shape[1]/2)
                    colourZeroedIn=frame[:,zeroedInEdges[0]:zeroedInEdges[1]] # all parts of image included
                    maskedZeroedIn=result[:,zeroedInEdges[0]:zeroedInEdges[1]] # only blue parts of image included
                    cv2.imshow("colourZeroedIn",colourZeroedIn)
                    foundPlate=self.findPlate(maskedZeroedIn,colourZeroedIn,centerList)
                    
                    cv2.imshow("License Plate",foundPlate)
                    cv2.waitKey(3)

                    foundPlate=cv2.resize(foundPlate,None, fx=5,fy=5) #scale to give some size to the image to make extracting contours easier
                    
                    character1=self.letterFilter(foundPlate[410:580,20:120])
                    character2=self.letterFilter(foundPlate[410:580,120:210])
                    character3=self.letterFilter(foundPlate[410:580,300:390])
                    character4=self.letterFilter(foundPlate[410:580,390:480])

                    first2Merge=np.concatenate((character1, character2), axis=1)
                    first3Merge=np.concatenate((first2Merge, character3), axis=1)
                    allMerge=np.concatenate((first3Merge, character4), axis=1)
                    cv2.imshow("Plate characters",allMerge)

                    self.prevCarTime=time.time()
                    print("im shleep")
                    #cv2.imshow("tight",self.locateCar(frame,centerList))
                    #self.founCounterPub.publish(self.foundCounter)
            #After I detect the car, I need to zero in on the license plate
            #After this find various homographies for stopping positions, train neural net (blue and transform license "paltes to train)
            #test together
        else:
            print("keep going")
        
        cv2.imshow("car?",frame2)
        cv2.waitKey(3)

    def locateCar(self,borders,xMid):
        avgY=int(sum([center[1] for center in borders])/2)
        avgX=int(sum([center[0] for center in borders])/2)
        topOffset=100
        bottomOffset=100
        top=avgY-topOffset
        bottom=avgY+bottomOffset
        bigContourOffset=0
        smallContourOffset=0

        if avgX>xMid:
            leftEdge=min([center[0] for center in borders])+bigContourOffset
            rightEdge=max([center[0] for center in borders])-smallContourOffset
        else:
            leftEdge=min([center[0] for center in borders])+smallContourOffset
            rightEdge=max([center[0] for center in borders])-bigContourOffset
        return (leftEdge,rightEdge)
    
    def findPlate(self, colourMasked, img, centers):
        
        # convert the blue only image to gray scale to find contours
        img_gray = cv2.cvtColor(colourMasked, cv2.COLOR_BGR2GRAY)
        # plt.imshow(img_gray,cmap='gray')
        # plt.show()
        threshold = 1 
        _, img_bin = cv2.threshold(img_gray, threshold, 255, cv2.THRESH_BINARY) #apply binary mask


        centerY=int(sum([center[1] for center in centers])/2)
        relevantRow=img_bin[centerY]
        # moving avg window size
        windowSize=5
        i=0
        prevAvg=0
        
        # detecting edges by moving average
        while i<len(relevantRow)-windowSize+1:
            window=relevantRow[i:i+windowSize]
            windowAvg=sum(window)/windowSize
            if not windowAvg and prevAvg:
                break
            i+=1
            prevAvg=windowAvg

        g=len(relevantRow)
        prevAvg=0
        while g-windowSize>0:
            window=relevantRow[g-windowSize:g]
            windowAvg=sum(window)/windowSize
            if not windowAvg and prevAvg:
                break
            g-=1
            prevAvg=windowAvg
    
        return img[centerY-50:centerY+100,i:g]

    def letterFilter(self,img):
        #colour thresholding
        lower_blue = np.array([90, 65, 0])
        upper_blue = np.array([255, 255, 255])

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        result = cv2.bitwise_and(img, img, mask = mask)

        return result
        
    def contourCenterX(contour):
        M=cv2.moments(contour)
        return int(M["m10"]/M["m00"])

    def boundedRectContour(contour, bigPicture):
        x,y,w,h = cv2.boundingRect(contour)
        return cv2.resize(bigPicture[y: y+h, x: x+w],(50,50))

    def getAlphaNumContours(self,image, sortedContours):
        centerList=[]
        sortedContours=sorted(sortedContours, key=self.contourCenterX) 
        letters=list(map(functools.partial(self.boundedRectContour,bigPicture=image),sortedContours[:2]))
        numbers=list(map(functools.partial(self.boundedRectContour,bigPicture=image),sortedContours[2:]))
        return letters,numbers

    def sendPlate(self, plateString, plateID):
        msg=f"TeamRed,multi21,{plateID},{plateString}"
        rospy.loginfo(msg)
        self.plate_pub.publish(msg)

def main(args):
    rospy.init_node("license_detector",anonymous=True)
    ic = plateProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down plate detection")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)