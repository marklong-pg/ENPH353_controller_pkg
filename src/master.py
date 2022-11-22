#! /usr/bin/env python3

import rospy
import sys
import time
from std_msgs.msg import String, Int8

class concertmaster:
    def __init__(self):
        self.drive_enb = rospy.Publisher('/drive_enb',Int8,queue_size=1)
        self.license_pub = rospy.Publisher('/license_plate', String, queue_size=1)
        rospy.init_node('concertmaster', anonymous=True)
        self.state = "starting"
    
    def click_timer(self,action):
        msg = f"TeamRed,multi21,{action},XXXX"
        rospy.loginfo(msg)
        self.license_pub.publish(msg)
        time.sleep(4)
        if action == 0:
            self.state = "start_drive"
        else:
            self.state = "shutdown"

# def start_timer():
#     pub = rospy.Publisher('/license_plate', String, queue_size=1)
#     rospy.init_node('master', anonymous=True)
#     rate = rospy.Rate(10)
#     started = False
#     while not rospy.is_shutdown():
#         if not started:
#             msg = "TeamRead,multi21,0,XXXX"
#             rospy.loginfo(msg)
#             pub.publish(msg)
#             started = True
#             rate.sleep()
#         else:
#             continue

# if __name__ == '__main__':
#     try:
#         start_timer()
#     except rospy.ROSInterruptException:
#         pass

def main(args):
    master = concertmaster()
    try:
        while not rospy.is_shutdown():
            if master.state == "starting":
                master.click_timer(0)
            if master.state == "start_drive":
                master.click_timer(-1)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)

    
    
    

