#!/usr/bin/env python3

import gym
from gym import wrappers
import gym_gazebo
import numpy
import random
import time
import time
import cv2
import gym
import math
import rospy
import roslaunch
import functools
import time
import numpy as np
import pickle
import os
# import imutils

import tensorflow as tf
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model
from tensorflow.keras import models
 

from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8
from time import sleep

from gym.utils import seeding

from matplotlib import pyplot as plt
from PIL import Image as im
from PIL import ImageEnhance
import sys
import csv

os.environ['CUDA_VISIBLE_DEVICES'] = "0"

#sess1 = tf.Session() 
# sess1=tf.compat.v1.Session()
# graph1 = tf.compat.v1.get_default_graph()
# set_session(sess1)


class plateProcessor:
    def __init__(self):

        foundCounter=0
        #self.founCounterPub=rospy.Publisher("/R1/foundCounter",int,queue_size=1)
        self.prevCarTime=0
        self.foundCounter=0
        self.startTime=time.time()
        self.imList=[]
        self.carCount=0
        self.saveTag=False

        self.cropDict={}
        self.cropDict[1]=[[0,21],[22,40],[50,67],[70,85]]
        self.cropDict[2]=[[0,21],[22,40],[50,67],[68,85]]
        self.cropDict[3]=[[0,21],[22,40],[52,69],[69,86]]
        self.cropDict[4]=[[2,20],[22,36],[50,66],[67,85]]
        self.cropDict[5]=[[0,21],[22,40],[50,68],[70,87]]
        self.cropDict[6]=[[0,21],[22,40],[50,67],[70,85]]

        plateFile=open("/home/fizzer/ros_ws/src/2022_competition/enph353/enph353_gazebo/scripts/plates.csv")
        plateReader=csv.reader(plateFile)

        self.plateList=[]
        for row in plateReader:
            self.plateList.append(row[0]+"_"+str(time.time()%1000))

        print(self.plateList)

        self.letterConvModel= tf.keras.models.load_model('/home/fizzer/ros_ws/src/controller_pkg/src/BlueOnlyLetterModel3/')
        self.numberConvModel= tf.keras.models.load_model('/home/fizzer/ros_ws/src/controller_pkg/src/BlueOnlyNumModelSimData2/')

        self.plate_pub=rospy.Publisher('/license_plate', String, queue_size=1)
        self.car_count = rospy.Publisher('/car_count',Int8,queue_size=1)
        self.drive_enb = rospy.Publisher('/drive_enb',Int8,queue_size=1)
        rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.carDetect,foundCounter,queue_size=1, buff_size=(1000000))

        # rospy.spin()
        # cv2.destroyAllWindows()

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
            #print("im shleep")
            # if(time.time()-self.prevCarTime>15):
            #     self.drive_enb.publish(1) 
            # if(0.2<=time.time()-self.prevCarTime<=0.5):
            #     toSave=frame
            #     imgToSave=im.fromarray(cv2.cvtColor(toSave, cv2.COLOR_BGR2RGB))
            #     imgToSave.save("/home/fizzer/PlateCapture/"+"lateP"+"_"+str(time.time())+".png")
            return

        if self.carCount==6:
            self.drive_enb.publish(0) 
            while True:
                i=1
    
        #blue thresholds for the hsv image, tuned for vehicle blue
        lower_blue = np.array([100, 125, 100])
        upper_blue = np.array([255, 255, 255])
        contourXDistThresh=250
        contourYDistThresh=100
        tooCloseXThresh=30
        
        #print("stay woke")

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
                            #print("putting g3")
                            carFound=True
                        centerList=[centerList[0],centerList[2]]
                elif abs(centerList[1][0]-centerList[0][0])<=contourXDistThresh and abs(centerList[1][1]-centerList[0][1])<=contourYDistThresh:
                    for center in centerList:
                        cv2.circle(frame2, center, 10, (0,0,255), -1)
                        carFound=True
                
                #save plate logic
                if carFound:
                    #bookkeeping
                    self.carCount+=1
                    self.car_count.publish(self.carCount)
                    print(self.carCount)
                    rgbFrame=cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    self.prevCarTime=time.time() 

                    #zero in on car
                    zeroedInEdges=self.locateCar(centerList,frame.shape[1]/2)
                    colourZeroedIn=frame[:,zeroedInEdges[0]:zeroedInEdges[1]] # all parts of image included

                    plateId=1 if self.carCount==6 else self.carCount+1

                    if plateId==5:
                        image=im.fromarray(cv2.cvtColor(colourZeroedIn,cv2.COLOR_BGR2RGB))
                        enhancer = ImageEnhance.Brightness(image)
                        im_output = enhancer.enhance(0.6)
                        undarkened=np.copy(colourZeroedIn)
                        colourZeroedIn=cv2.cvtColor(np.array(im_output),cv2.COLOR_RGB2BGR)
                    if plateId==4:
                        self.undarkened1=np.copy(colourZeroedIn)

                    try:
                        if plateId==4:
                            plate=self.findPlateV2(colourZeroedIn)
                            unwarped=self.warpPlateFour(plate)
                            image=im.fromarray(unwarped)
                            enhancer = ImageEnhance.Brightness(image)
                            plate = np.array(enhancer.enhance(0.6))
                        else :
                            plate=self.findPlateV2(colourZeroedIn)

                        cv2.imshow("plate",plate)
                        cv2.moveWindow("plate",0,0)
                        print(self.cropAndPredict(plate,plateId))
                        toSave=colourZeroedIn if plateId!=5 else undarkened
                        imgToSave=im.fromarray(cv2.cvtColor(toSave, cv2.COLOR_BGR2RGB))
                        imgToSave.save("/home/fizzer/AutomatedPlateCapture/FinalDrive/"+"P"+str(plateId)+"_"+self.plateList[plateId-1]+".png")
                        print("saving "+str(plateId))
                        if plateId==5:
                            imgToSave=im.fromarray(cv2.cvtColor(self.undarkened1, cv2.COLOR_BGR2RGB))
                            imgToSave.save("/home/fizzer/AutomatedPlateCapture/FinalDrive/"+"P"+str(plateId-1)+"_"+self.plateList[plateId-2]+".png")

                    except:
                        self.carCount=6
                        print("crash :(")

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
    
    def findBackOfCar(self, colourMasked, img, centers):
        
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

    def predictCharacter(self, character, num=0):
        GRAY=[89,89,89]

        character=cv2.cvtColor(character,cv2.COLOR_BGR2RGB)
        if num:
            desiredWidth=21
            desiredHeight=34
            top=desiredHeight-character.shape[0] if character.shape[0]<desiredHeight else 0
            right=desiredWidth-character.shape[1] if character.shape[1]<desiredWidth else 0
            padded=cv2.copyMakeBorder(character,top,0,right,0,cv2.BORDER_CONSTANT,value=GRAY)
            lower_blue = np.array([81, 80, 0])
            upper_blue = np.array([255, 255, 255])
            hsv = cv2.cvtColor(padded, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            onlyBlue = cv2.bitwise_and(padded, padded, mask = mask) 

            grayFiltered = cv2.cvtColor(onlyBlue, cv2.COLOR_RGB2GRAY)
            contours, _ = cv2.findContours(grayFiltered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            sortedContours=sorted(contours,key=cv2.contourArea,reverse=True)
            cv2.drawContours(onlyBlue, sortedContours[1:],
                            -1, (0, 0, 0), -1)
            toPredict=onlyBlue
        else:
            desiredWidth=21
            desiredHeight=32
            top=desiredHeight-character.shape[0] if character.shape[0]<desiredHeight else 0
            right=desiredWidth-character.shape[1] if character.shape[1]<desiredWidth else 0
            toPredict=cv2.copyMakeBorder(character,top,0,right,0,cv2.BORDER_CONSTANT,value=GRAY)
            padded=cv2.copyMakeBorder(character,top,0,right,0,cv2.BORDER_CONSTANT,value=GRAY)
            lower_blue = np.array([81, 80, 0])
            upper_blue = np.array([255, 255, 255])
            hsv = cv2.cvtColor(padded, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            onlyBlue = cv2.bitwise_and(padded, padded, mask = mask) 

            grayFiltered = cv2.cvtColor(onlyBlue, cv2.COLOR_RGB2GRAY)
            contours, _ = cv2.findContours(grayFiltered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            sortedContours=sorted(contours,key=cv2.contourArea,reverse=True)
            cv2.drawContours(onlyBlue, sortedContours[1:],
                            -1, (0, 0, 0), -1)
            toPredict=onlyBlue

        # cv2.imshow("To Predict",toPredict)
        # cv2.waitKey(5)
        
        toPredictResize=cv2.resize(toPredict,None,fx=5,fy=5)
        print(toPredictResize.shape)
        img_aug = tf.cast(toPredictResize, tf.float32)
        img_aug = np.expand_dims(img_aug, axis=0)

        if num:
            NN_prediction = self.numberConvModel.predict(img_aug)[0]
            predictedNumber=np.argmax(NN_prediction)
            return (str(predictedNumber),toPredict)
        else:
            NN_prediction = self.letterConvModel.predict(img_aug)[0]
            predictedNumber=np.argmax(NN_prediction)
            return (chr(predictedNumber+65),toPredict)
        
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


    def findPlateV2(self, carBack):

        lower_blue = np.array([100, 125, 100])
        upper_blue = np.array([255, 255, 255])
        hsv = cv2.cvtColor(carBack, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        notMask=cv2.bitwise_not(mask)
        onlyBlue = cv2.bitwise_and(carBack, carBack, mask = mask)
        notResult = cv2.bitwise_and(carBack, carBack, mask = notMask)
        # cv2.imshow("result", result)
        # cv2.waitKey(3)

        grayFiltered = cv2.cvtColor(onlyBlue, cv2.COLOR_RGB2GRAY)
        contours, _ = cv2.findContours    (grayFiltered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        sortedContours=sorted(contours, key=cv2.contourArea, reverse=True)
        
        centerList=[]
        for i in range(2):
            M = cv2.moments(sortedContours[i])
            centerThree = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            centerList.append(centerThree)
        
        notBluePlateFind=self.findBackOfCar(onlyBlue, notResult, centerList)
        plateFind=self.findBackOfCar(onlyBlue, carBack,centerList)

        # cv2.imshow("notPlateFind", notPlateFind)
        # cv2.waitKey(3)
        
        plateOnly=self.plateFilter(notBluePlateFind, plateFind)

        return plateOnly 
    
    def plateFilter(self, carBack, colourCarBack):
        #carBack=carDetectFrames[0][0][2]
        if self.carCount==3:     
            upper_plateGray = np.array([153, 66, 187])
            lower_plateGray = np.array([104, 0, 121])
        else:
            lower_plateGray = np.array([0, 2, 31])
            upper_plateGray = np.array([174, 20, 115])
            
        hsv = cv2.cvtColor(carBack, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_plateGray, upper_plateGray) 
        result = cv2.bitwise_and(carBack, carBack, mask = mask)
        
        # cv2.imshow("plate filter mask",result)
        # cv2.waitKey(2)
        
        grayFiltered = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(grayFiltered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        sortedContours=sorted(contours, key=cv2.contourArea, reverse=True)

        x,y,w,h = cv2.boundingRect(sortedContours[0])
        

        return colourCarBack[y: y+h, x: x+w]

    def cropAndPredict(self, plate, plateId):
        cropList=self.cropDict[plateId]
        plateString=""

        characterList=[]

        for i in range(2):
            predictReturn=self.predictCharacter(plate[:,cropList[i][0]:cropList[i][1]],False)
            plateString+=predictReturn[0]
            characterList.append(predictReturn[1])

        for i in range(2):
            predictReturn=self.predictCharacter(plate[:,cropList[i+2][0]:cropList[i+2][1]],True)
            plateString+=predictReturn[0]
            characterList.append(predictReturn[1])

        predictionCrop=np.concatenate([cv2.copyMakeBorder(characterList[i],0,0,0,5,borderType=cv2.BORDER_CONSTANT,value=(255,255,255)
        ) for i in range(2)],axis=1)
        cv2.imshow("predictionCrop",cv2.cvtColor(predictionCrop,cv2.COLOR_RGB2BGR))
        cv2.moveWindow("predictionCrop",x=0,y=200)
        predictionCrop=np.concatenate([cv2.copyMakeBorder(characterList[i],0,0,0,5,borderType=cv2.BORDER_CONSTANT,value=(255,255,255)
        ) for i in range(2,4)],axis=1)
        cv2.imshow("predictionCrop1",cv2.cvtColor(predictionCrop,cv2.COLOR_RGB2BGR))
        cv2.moveWindow("predictionCrop1",x=0,y=400)
        cv2.waitKey(5)

        self.sendPlate(plateString,plateId)

        return plateString

    def warpPlateFour(self, plate):
        pt_A = [0, 0]
        pt_B = [2, plate.shape[0]-2]
        pt_C = [plate.shape[1], 5]
        pt_D = [plate.shape[1], plate.shape[0]]

        inputPts =np.float32([pt_A, pt_B, pt_C, pt_D])
        outputPts = np.float32([[0, 0],
                                [0,20],
                                [87, 0],
                                [87, 20]])

        M = cv2.getPerspectiveTransform(inputPts,outputPts)
        out = cv2.warpPerspective(plate,M,(87, 22),flags=cv2.INTER_LINEAR)
        return out




def main(args):
    rospy.init_node("license_detector",anonymous=True)
    sleep(2)
    ic = plateProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down plate detection")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)