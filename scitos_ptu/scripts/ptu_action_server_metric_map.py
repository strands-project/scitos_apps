#! /usr/bin/env python

import roslib; 
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

import scitos_ptu.msg
from sensor_msgs.msg import *
from std_msgs.msg import String
from flir_pantilt_d46.msg import *
import threading
import math
import time


def clampValue(val, max_val, min_val):
	if (val>max_val):
		val = max_val
	if (val<min_val):
		val = min_val
	return val

class PTUServer:
  def __init__(self):
    rospy.init_node('ptu_action_server_metric_map')
    self.log_pub = rospy.Publisher('/ptu/log', String)

    self.server = actionlib.SimpleActionServer('ptu_pan_tilt_metric_map', scitos_ptu.msg.PanTiltAction, self.execute, False)
    self.server.register_preempt_callback(self.preemptCallback)
    self.server.start()
    rospy.loginfo('Ptu action server started')
 
    self.aborted = False
    self.preempted = False
    self.preempt_timeout = 0.3 # seconds
    self.pan_tilt_timeout = 10 # seconds

    self.feedback = scitos_ptu.msg.PanTiltFeedback()
    self.result = scitos_ptu.msg.PanTiltResult()
    self.result.success = True

    self.max_pan = 160.0 # degrees
    self.min_pan = -160.0 # degrees

    self.max_tilt = 30.0 # degrees
    self.min_tilt = -30.0 # degrees	
    self.preempt_lock = threading.Lock()


    self.ptugoal = flir_pantilt_d46.msg.PtuGotoGoal()
    self.ptugoal.pan_vel = 60
    self.ptugoal.tilt_vel = 60

    self.client = actionlib.SimpleActionClient("SetPTUState", flir_pantilt_d46.msg.PtuGotoAction)
    self.client.wait_for_server()
    print 'Ptu client created'

  def execute(self, goal):
    
    panstart = goal.pan_start
    if (panstart < self.min_pan):
        panstart = self.min_pan
	print 'Warning, panstart value outside range. Clamping to ',panstart

    panstep = goal.pan_step

    panend = goal.pan_end
    if (panend > self.max_pan):
        panend = self.max_pan
	print 'Warning, panend value outside range. Clamping to ',panend

    tiltstart = goal.tilt_start

    if (tiltstart < self.min_tilt):
        tiltstart = self.min_tilt
	print 'Warning, tiltstart value outside range. Clamping to ',tiltstart

    tiltstep = goal.tilt_step

    tiltend = goal.tilt_end
    if (tiltend > self.max_tilt):
	tiltend = int(self.max_tilt)
	print 'Warning, tiltend value outside range. Clamping to ',tiltend

    print 'Starting pan tilt action ',panstart, panstep, panend, tiltstart, tiltstep, tiltend
    # start position
    self.log_pub.publish("start_sweep")
    time.sleep(1) # sleep for 1 second here
    self.log_pub.publish("sweep_parameters " +str(panstart)+" "+str(panstep)+" "+str(panend)+" "+str(tiltstart)+" "+str(tiltstep)+" "+str(tiltend))
    self.ptugoal.pan = -panstart
    self.ptugoal.tilt = -tiltstart	    
    self.preempted = False
    self.aborted = False
    self.client.send_goal(self.ptugoal)
    self.client.wait_for_result()
    reverseSweep = False
    
    for j in range(tiltstart, tiltend+tiltstep, tiltstep):
       if self._get_preempt_status() or self.aborted:
                break;	 
       for i in range(panstart, panend+panstep, panstep):
		if self._get_preempt_status() or self.aborted:
                	break;	
		# set the tilt angle considering whether we are moving the pantilt backwards
		if not reverseSweep:
    			self.ptugoal.pan = -i
		else:
    			self.ptugoal.pan = -(-i+panstart+panend)

		self.ptugoal.tilt =-j

		# check limits
		self.ptugoal.pan = clampValue(self.ptugoal.pan, -panstart, -panend)
		self.ptugoal.tilt = clampValue(self.ptugoal.tilt, -tiltstart, -tiltend)

    		self._sendPTUGoal(self.ptugoal)
		# keep this position for logging
          	self.log_pub.publish("start_position")
		time.sleep(2) # sleep for 2 seconds here
		self.log_pub.publish("end_position")
		self.feedback.feedback_ptu_pose.position = [self.ptugoal.pan, self.ptugoal.tilt]
		self.server.publish_feedback(self.feedback)
       reverseSweep = not reverseSweep

    self.ptugoal.pan = 0
    self.ptugoal.tilt =0
    self._sendPTUGoal(self.ptugoal)
    self.log_pub.publish("end_sweep")
    
    if self._get_preempt_status():
		self.log_pub.publish("preempted")
		self.result.success = False
		self.server.set_preempted(self.result)
    elif self.aborted:
		self.log_pub.publish("aborted")
		self.result.success = False
		self.server.set_aborted(self.result)
    else:
		self.result.success = True
		self.server.set_succeeded(self.result)

  def _sendPTUGoal(self, ptu_goal):
	time_waited = 0
	self.client.send_goal(self.ptugoal)
	self.client.wait_for_result(rospy.Duration(self.preempt_timeout))
	status= self.client.get_state()
	while not status == GoalStatus.SUCCEEDED and not time_waited > self.pan_tilt_timeout and not self._get_preempt_status():
		time_waited += self.preempt_timeout
		self.client.wait_for_result(rospy.Duration(self.preempt_timeout))
		status= self.client.get_state()

	if self._get_preempt_status():
		# this action server has been preempted; preempt the other one as well
		self.client.cancel_goal()
 	elif time_waited > self.pan_tilt_timeout or status != GoalStatus.SUCCEEDED:
		# didn't manage to reach the PTU position
		self.client.cancel_goal()
		self.aborted = True		

  def _get_preempt_status(self):
	self.preempt_lock.acquire()
	preempted = self.preempted
	self.preempt_lock.release()
	return preempted	

  def preemptCallback(self):
    self.preempt_lock.acquire()
    self.preempted = True
    self.preempt_lock.release()


if __name__ == '__main__':
  server = PTUServer()
  rospy.spin()
