#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Joy
from scitos_teleop.msg import action_buttons


class HeadTeleop():
	"A class to command the Scitos head from a joystick"

	PAN_INCREMENT=2
	TILT_INCREMENT=1
	MAX_PAN=80
	MIN_PAN=-80
	MAX_TILT=10
	MIN_TILT=-10
	MAX_PAN_INVERS=MAX_PAN-180
	MIN_PAN_INVERS=MIN_PAN-180

	def __init__(self):
		rospy.init_node('teleop_head')
		self.pub = rospy.Publisher('/head/commanded_state', JointState)
		rospy.Subscriber("/teleop_joystick/joy", Joy, self.callback) 
		rospy.Subscriber("/teleop_joystick/action_buttons", action_buttons, self.button_callback) 
		rospy.logdebug(rospy.get_name() + " setting up")
		self.invers=False;
		self.currentPan=0
		self.currentTilt=0
		self.head_command = JointState() 
		self.head_command.name=["HeadPan", "HeadTilt"] 
		self.head_command.position=[self.currentPan, self.currentTilt]
		self.eyelid_command = JointState() 
		self.eyelid_command.name=["EyeLids"]
		self.eye_command = JointState()
		self.eye_command.name=["EyesTilt", "EyesPan"]
		rospy.loginfo("Start")

	def callback(self, joy): 
		
		### Head pan/tilt control ###
		# Reset head to 0,0 if top right shoulder button is pressed		
		if(joy.buttons[5]):
			self.currentPan=0
			self.currentTilt=0
			if(self.invers):
				self.currentPan=-180
		
		rospy.logdebug(rospy.get_name() + ": I heard %s" % joy) 
		# print joy.axes[3:5] 

		# update value from axes
		self.currentPan += joy.axes[3] * self.PAN_INCREMENT
		self.currentTilt += joy.axes[4] * self.TILT_INCREMENT

		rospy.loginfo(self.currentPan)	

		# threshold value
		if(not self.invers):
			rospy.loginfo("not Invers!")
			self.currentPan = self.currentPan if self.currentPan < self.MAX_PAN else self.MAX_PAN
			self.currentPan = self.currentPan if self.currentPan > self.MIN_PAN else self.MIN_PAN
		else:
			#rospy.loginfo(self.currentPan-180)
			self.currentPan = self.currentPan if self.currentPan < self.MAX_PAN_INVERS else self.MAX_PAN_INVERS
			self.currentPan = self.currentPan if self.currentPan > self.MIN_PAN_INVERS else self.MIN_PAN_INVERS

		self.currentTilt = self.currentTilt if self.currentTilt < self.MAX_TILT else self.MAX_TILT
		self.currentTilt = self.currentTilt if self.currentTilt > self.MIN_TILT else self.MIN_TILT

		rospy.loginfo(self.currentPan)	
	
		#update command
		self.head_command.position=[self.currentPan, self.currentTilt] 

		### Eyes pan/tilt control ###
		self.eye_command.position=[joy.axes[4]*100, joy.axes[3]*100]

		### Eye lids control ###
		self.eyelid_command.position=[(joy.axes[5]+1)*50]

		# publish
		self.pub.publish(self.head_command)
		self.pub.publish(self.eye_command)
		self.pub.publish(self.eyelid_command)

	def button_callback(self, action_buttons):
		if(action_buttons.Y):
			rospy.loginfo("Buttons!")				
			if(self.invers):
				self.invers=False
				self.currentPan = 0
			else:			
				self.invers=True
				self.currentPan = -180

if __name__ == '__main__':
    head_teleop = HeadTeleop()
    rospy.spin()
