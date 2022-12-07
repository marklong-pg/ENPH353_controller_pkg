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
        self.plate_count_sub = rospy.Subscriber("/car_count",Int8,self.car_count_printer)
        self.inner_trig_sub = rospy.Subscriber('/inner_trigger',Int8,self.inner_loop_trigger)
        rospy.init_node('concertmaster', anonymous=True)
        self.state = "trans_to_inner"

    def car_count_printer(self,msg):
        if msg.data == 4:
            self.drive_enb.publish(1)
        elif msg.data == 6:
            self.state = "trans_to_inner"
            return

    def inner_loop_trigger(self,msg):
        # self.drive_enb.publish(3)
        time.sleep(3)
        print("Master Wakes")
        self.drive_enb.publish(4)
    
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

            elif master.state == "start_drive":
                print("Starting Drive")
                time.sleep(3)                       
                master.drive_enb.publish(10) # initial orientation
                time.sleep(5)
                master.drive_enb.publish(1) # start driving
                master.state = "outer_loop"
            
            elif master.state == "outer_loop":
                continue

            elif master.state == "trans_to_inner":
                time.sleep(2.4)
                master.drive_enb.publish(3)
                print("Transitioning to inner loop")
                time.sleep(3.5)
                # master.drive_enb.publish(0)
                # time.sleep(1)
                master.drive_enb.publish(6)
                print("Inner loop drive activated")
                master.state = "idle"

            elif master.state == "idle":
                continue

            elif master.state == "end":
                print("Ending Drive")
                master.click_timer(-1)
                master.drive_enb.publish(0)
                master.state = "idle"
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)