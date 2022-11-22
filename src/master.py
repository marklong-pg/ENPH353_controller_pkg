#! /usr/bin/env python3

import rospy
import sys
import time
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist

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
        time.sleep(2)      


def main(args):
    master = concertmaster()
    try:
        while not rospy.is_shutdown():
            if master.state == "starting":
                print("Master node startup successful!")
                master.click_timer(0)
                master.state = "start_drive"

            if master.state == "start_drive":
                print("Starting Drive")
                master.drive_enb.publish(10) # initial orientation
                time.sleep(5)
                master.drive_enb.publish(1) # start driving
                master.state = "in_drive"
            
            if master.state == "in_drive":
                continue

            if master.state == "end":
                print("Ending Drive")
                master.drive_enb.publish(0)
                master.click_timer(-1)
                master.state = "_"
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)