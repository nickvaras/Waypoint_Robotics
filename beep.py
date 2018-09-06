#!/usr/bin/env python
from __future__ import print_function
import rospy 
import sys
from pyModbusTCP.client import ModbusClient
from geometry_msgs.msg import Twist
from waypoint_ui.srv import ModbusSetDigitalOutput
import argparse

class beep(object):
    def __init__(self,delay):
        self.pub = rospy.Publisher('/vector/navigation/cmd_vel', Twist, queue_size=10)
        self.vel_value=None
        self.for_waiting=0
        self.stop_value=Twist()
        self.stop_value.linear.x=0
        self.stop_value.linear.y=0
        self.stop_value.linear.z=0
        self.stop_value.angular.x=0
        self.stop_value.angular.y=0
        self.stop_value.angular.z=0
        self.time_when_stopped=Time=None
        self.time_when_started_moving=None
        self.delay=delay
        rospy.Subscriber('/vector/navigation/cmd_vel_no_warning', Twist, self.callback, queue_size=10)

    def set_digital_output(self,io_name, value):
        rospy.wait_for_service('/waypoint_db/retrieve_waypoint', timeout=5.0)
        self.set_output = rospy.ServiceProxy('modbus_manager/set_digital_output', ModbusSetDigitalOutput)
        return  self.set_output(io_name,value)

    def check_if_moving(self):
        if self.vel_value.linear.x!=0 or self.vel_value.linear.y!=0 or self.vel_value.linear.z!=0 or self.vel_value.angular.x!=0 or self.vel_value.angular.y!=0 or self.vel_value.angular.z!=0:
            self.in_motion=1
            
        elif self.vel_value.linear.x==0 or self.vel_value.linear.y==0 or self.vel_value.linear.z==0 or self.vel_value.angular.x==0 or self.vel_value.angular.y==0 or self.vel_value.angular.z==0:
            self.in_motion=0
       
    def callback(self,data):
        self.time_when_started_moving =rospy.get_rostime()
        self.vel_value=Twist()
        self.vel_value = data
        self.check_if_moving()
        if self.for_waiting==0:
            # self.set_digital_output("y1",True)
            rospy.sleep(int(self.delay))
            
        self.pub.publish(self.vel_value)
    
        if self.in_motion==0:
            self.time_when_stopped=rospy.get_rostime()
            diff = self.time_when_stopped.secs - self.time_when_started_moving.secs
            self.for_waiting=0
            if diff>=5:
                # self.set_digital_output("y1",False)
                rospy.loginfo("Parked")
        else:
            self.for_waiting = 1
            rospy.loginfo("beep")
if __name__ == "__main__":

    rospy.init_node('beep')
    parser = argparse.ArgumentParser(description='delay parser')
    parser.add_argument('--delay', action="store", dest='delay', default=3)
    args=parser.parse_args()
    print (args.delay)
    beep(args.delay)
    
    rospy.spin()

    