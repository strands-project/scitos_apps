#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Joy


class HeadTeleop():
	"A class to command the Scitos head from a joystick"

	PAN_INCREMENT=2
	TILT_INCREMENT=1
	MAX_PAN=80
	MIN_PAN=-80
	MAX_TILT=10
	MIN_TILT=-10

	def __init__(self):
		rospy.init_node('teleop_head')
		self.pub = rospy.Publisher('/head/commanded_state', JointState)
		rospy.Subscriber("/teleop_joystick/joy", Joy, self.callback) 
		rospy.logdebug(rospy.get_name() + " setting up")
		self.currentPan=0
		self.currentTilt=0
		self.head_command = JointState() 
		self.head_command.name=["HeadPan", "HeadTilt"] 
		self.head_command.position=[self.currentPan, self.currentTilt]
		self.eyelid_command = JointState() 
		self.eyelid_command.name=["EyeLidLeft", "EyeLidRight"]
		self.eye_command = JointState()
		self.eye_command.name=["EyesTilt", "EyesPan"]

	def callback(self, joy): 
		
		### Head pan/tilt control ###
		# Reset head to 0,0 if top right shoulder button is pressed		
		if(joy.buttons[5]):
			self.currentPan=0
			self.currentTilt=0
		
		rospy.logdebug(rospy.get_name() + ": I heard %s" % joy) 
		# print joy.axes[3:5] 

		# update value from axes
		self.currentPan += joy.axes[3] * self.PAN_INCREMENT
		self.currentTilt += joy.axes[4] * self.TILT_INCREMENT

		# threshold value
		self.currentPan = self.currentPan if self.currentPan < self.MAX_PAN else self.MAX_PAN
		self.currentPan = self.currentPan if self.currentPan > self.MIN_PAN else self.MIN_PAN
		self.currentTilt = self.currentTilt if self.currentTilt < self.MAX_TILT else self.MAX_TILT
		self.currentTilt = self.currentTilt if self.currentTilt > self.MIN_TILT else self.MIN_TILT
	
		#update command
		self.head_command.position=[self.currentPan, self.currentTilt] 

		### Eyes pan/tilt control ###
		self.eye_command.position=[joy.axes[4]*100, joy.axes[3]*100]

		### Eye lids control ###
		self.eyelid_command.position=[(joy.axes[5]+1)*50]*2

		# publish
		self.pub.publish(self.head_command)
		self.pub.publish(self.eye_command)
		self.pub.publish(self.eyelid_command)

if __name__ == '__main__':
    head_teleop = HeadTeleop()
    rospy.spin()
