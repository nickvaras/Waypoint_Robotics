#!/usr/bin/env python
import functools
import rospy 
from pyModbusTCP.client import ModbusClient
from geometry_msgs.msg import Twist
from waypoint_ui.srv import ModbusSetDigitalOutput


class HMI(object):
    def __init__(self):
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
        rospy.Subscriber('/vector/navigation/cmd_vel_no_warning', Twist, self.callback, queue_size=10)

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
            rospy.sleep(5)
        self.pub.publish(self.vel_value)
    
        if self.in_motion==0:
            self.time_when_stopped=rospy.get_rostime()
            diff = self.time_when_stopped.secs - self.time_when_started_moving.secs
            self.for_waiting=0
            if diff>=5:
                rospy.loginfo("Parked")
        else:
            self.for_waiting = 1
if __name__ == "__main__":
    rospy.init_node('beep')
    try:
        HMI()
    except rospy.ROSInterruptException:
        raise e
    
    rospy.spin()

    