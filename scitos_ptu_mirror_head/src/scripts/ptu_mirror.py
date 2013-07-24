#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math


class HeadTeleop():
	"A class to command the PTU to miror the movements of the Scitos head"
	MAX_PAN=80
	MIN_PAN=-80
	MAX_TILT=10
	MIN_TILT=-10
	
	def __init__(self):
		rospy.init_node('ptu_mirror')
		self.pub = rospy.Publisher('/ptu/cmd', JointState)
		rospy.Subscriber("/head/actual_state", JointState, self.callback) 
		#rospy.Subscriber("/teleop_joystick/action_buttons", action_buttons, self.button_callback) 
		rospy.logdebug(rospy.get_name() + " setting up")
		#self.invers=False;
		self.ptu_command = JointState() 
		self.ptu_command.name=["pan", "tilt"] 
		self.ptu_command.position=[0,0]
		self.ptu_command.velocity=[1.0]
		rospy.loginfo("Start")
	
	def callback(self, head_state):
		self.ptu_command.position = []
		for index, item in enumerate(head_state.name):
			if item == "HeadPan":
				pan = head_state.position[index] if head_state.position[index] < MAX_PAN else MAX_PAN
				pan = head_state.position[index] if head_state.position[index] > MIN_PAN else MIN_PAN
				self.ptu_command.position.append(pan*(math.pi/180))
			if item == "HeadTilt":
				tilt = head_state.position[index] if head_state.position[index] < MAX_TILT else MAX_TILT
				tilt = head_state.position[index] if head_state.position[index] > MIN_TILT else MIN_TILT
				self.ptu_command.position.append(-tilt*(math.pi/180))
		self.pub.publish(self.ptu_command)

if __name__ == '__main__':
    head_teleop = HeadTeleop()
    rospy.spin()
