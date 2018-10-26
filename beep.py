#!/usr/bin/env python
from __future__ import print_function
import rospy 
import sys
from geometry_msgs.msg import Twist
from waypoint_ui.srv import ModbusSetDigitalOutput
import argparse

class AudioVisualWarningAndDelay(object):
    def __init__(self,start_delay,park_delay):
        
        self.for_waiting=0
        self.time_when_stopped_moving=None
        self.time_when_started_moving=None
        self.start_delay=start_delay
        self.park_delay=park_delay
        self.previous_command_was_zero=False

        # Initialize the service proxy
        try:
            rospy.wait_for_service('modbus_manager/set_digital_output', timeout=5.0)
        except:
            rospy.logerr("Failed to connect to digital IO service, exiting...")
            sys.exit(1)

        self.set_digital_output = rospy.ServiceProxy('modbus_manager/set_digital_output', ModbusSetDigitalOutput)

        # Initialize the publishers and subscribers
        rospy.Subscriber('/vector/navigation/cmd_vel_no_warning', Twist, self.cmd_vel_callback, queue_size=10)
        self.velocity_publisher = rospy.Publisher('/vector/navigation/cmd_vel', Twist, queue_size=10)
              
    def cmd_vel_callback(self,velocity_command):
        self.time_when_started_moving =rospy.get_rostime()        
        if self.for_waiting==0:
            self.set_digital_output("y1",True)
            rospy.sleep(int(self.start_delay))   ###############
            
        if not ((velocity_command.linear.x == 0) and (velocity_command.linear.y == 0) and (velocity_command.angular.z == 0)):
            self.time_when_stopped_moving=rospy.get_rostime()
            
            self.for_waiting=0
            if (self.time_when_stopped_moving.secs - self.time_when_started_moving.secs) >= self.park_delay:
                self.set_digital_output("y1",False)
                rospy.loginfo("Parked")
        else:
            self.for_waiting = 1
            rospy.loginfo("beep")

        self.velocity_publisher.publish(velocity_command)
        #self.velocity_publisher.publish(Twist())   # publishes zero velocity

if __name__ == "__main__":
        if len(sys.argv) != 3:
            print("Usage: audio_visual_warning_and_delay start_delay park_delay")
        else:
            rospy.init_node('audio_visual_warning_and_delay')
            AudioVisualWarningAndDelay(sys.argv[1],sys.argv[2])
            rospy.spin()
