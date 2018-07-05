#!/usr/bin/env python
import rospy
import logging
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from behavior_common.msg import CommandState

def callback(data):
    msg = CommandState()

    # we only allow one behavior state at a time
    if data.buttons[2] == 1: # Red B button - Stop current behavior
        msg.commandState = "STOP"
    	pub_behavior.publish(msg) 
    elif data.buttons[1] == 1: # Green A button
        msg.commandState = "FOLLOW_ME"
    	pub_behavior.publish(msg) 
    elif data.buttons[0] == 1: # Blue X button
        msg.commandState = "TURN_LEFT"
    	pub_behavior.publish(msg) 
    elif data.buttons[3] == 1: # Yellow Y button
    	pub_behavior.publish('TURN_RIGHT') 
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    # Publish an action for the behavior engine to handle
    pub_behavior = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)
    listener()
