#! /usr/bin/env python

import roslib; 
import rospy
import actionlib

from scitos_ptu.msg import *
from sensor_msgs.msg import *
import math
import time

class PTUServer:
  def __init__(self):
    rospy.init_node('ptu_actionserver')
    self.pub = rospy.Publisher('/ptu/cmd', JointState)
    self.state=rospy.Subscriber("/ptu/state", JointState, self.head_state_cb)

    self.server = actionlib.SimpleActionServer('ptu_pan_tilt', PanTiltAction, self.execute, False)
    self.server.start()
 
    self.reached_epsilon = math.radians(1.0)
    self.reached = False

    self.ptu_command = JointState()
    self.ptu_command.name=["pan", "tilt"]
    self.ptu_command.position=[0.0, 0.0]
    self.ptu_command.velocity=[1.0,1.0]

    self.max_pan = 120.0 # degrees
    self.min_pan = -120.0 # degrees

    self.max_tilt = 30.0 # degrees
    self.min_tilt = -30.0 # degrees

  def head_state_cb(self,msg):
	if (math.fabs(msg.position[0] - self.ptu_command.position[0]) < self.reached_epsilon) and (math.fabs(msg.position[1] - self.ptu_command.position[1]) < self.reached_epsilon):
		self.reached = True
	else:
		self.reached = False

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

    tiltstep = int(goal.target_ptu_pose.position[4])

    tiltend = goal.tilt_end
    if (tiltend > self.max_tilt):
	tiltend = int(self.max_tilt)
	print 'Warning, tiltend value outside range. Clamping to ',tiltend

    # start position
    self.ptu_command.position=[math.radians(-panstart),math.radians(-tiltstart)]
    self.pub.publish(self.ptu_command)
    self.reached = False

    start_time = time.time()
    elapsed_time = 0
    while (not self.reached and elapsed_time < 3):
	time.sleep(0.1)
	elapsed_time = time.time()-start_time
    
    for i in range(panstart, panend, panstep):
       for j in range(tiltstart, tiltend, tiltstep):
          self.ptu_command.position=[math.radians(-i),math.radians(-j)]
          self.pub.publish(self.ptu_command)
    	  self.reached = False

	  start_time = time.time()
    	  elapsed_time = 0
	  while (not self.reached and elapsed_time < 3):
		time.sleep(0.1)
		elapsed_time = time.time()-start_time

    self.ptu_command.position=[math.radians(0),math.radians(0)]
    self.pub.publish(self.ptu_command)
    self.reached = False
    
    start_time = time.time()
    elapsed_time = 0
    while (not self.reached and elapsed_time < 3):
	time.sleep(0.1)
	elapsed_time = time.time()-start_time
    
    self.server.set_succeeded()

if __name__ == '__main__':
  server = PTUServer()
  rospy.spin()
