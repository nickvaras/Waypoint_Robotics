#!/usr/bin/env python
from __future__ import print_function
import rospy 
import sys
from geometry_msgs.msg import Twist

class AudioVisualWarningAndDelay(object):
    def __init__(self,move_delay,park_delay):       

        # Initialize the service proxy
        try:
            rospy.wait_for_service('modbus_manager/set_digital_output', timeout=5.0)
            self.set_digital_output = rospy.ServiceProxy('modbus_manager/set_digital_output', ModbusSetDigitalOutput)
        except:
            rospy.logerr("Failed to connect to digital IO service, exiting...")
            sys.exit(1)

        # Initialize the publishers and subscribers
        rospy.Subscriber('/vector/navigation/cmd_vel_no_warning', Twist, self.cmd_vel_callback, queue_size=10)
        self.velocity_publisher = rospy.Publisher('/vector/navigation/cmd_vel', Twist, queue_size=10)

        self.state = "stopped"    # states are:  moving, just_stopped, stopped and about_to_move
        self.time_since_state_change = None
        self.control_period = 0.1
        self.move_delay = move_delay
        self.park_delay = park_delay

        while not rospy.is_shutdown():
            if self.state == "just_stopped":
                if self.time_since_state_change > park_delay:
                    self.state = "stopped"
                    self.set_warnings_off()

            elif self.state == "about_to_move":
                self.set_warnings_on()
                if self.time_since_state_change > move_delay:
                    self.state = "moving" 

    def set_warnings_off(self):
        self.set_digital_output("y1",False)

    def set_warnings_on(self):
        self.set_digital_output("y1",True)
              
    def cmd_vel_callback(self,velocity_command):

        if ((velocity_command.linear.x == 0) and (velocity_command.linear.y == 0) and (velocity_command.angular.z == 0)):  # i.e., be still
            if self.state == "moving":
                self.state = "just_stopped"
                self.time_since_state_change = rospy.get_time()
            elif self.state == "about_to_move":
                self.state = "stopped"
            self.velocity_publisher.publish(velocity_command)
        
        else:  # i.e., move
            if self.state == "moving":
                self.velocity_publisher.publish(velocity_command)
            elif self.state == "just_stopped":
                self.velocity_publisher.publish(velocity_command)
                self.state = "moving"
            elif self.state == "stopped":
                self.velocity_publisher.publish(Twist())
                self.state = "about_to_move"
                self.time_since_state_change = rospy.get_time()
            elif self.state == "about_to_move":
                self.velocity_publisher.publish(Twist())

if __name__ == "__main__":
        if len(sys.argv) != 3:
            print("Usage: audio_visual_warning_and_delay start_delay park_delay")
        else:
            rospy.init_node('audio_visual_warning_and_delay')
            AudioVisualWarningAndDelay(sys.argv[1],sys.argv[2])
            rospy.spin()
