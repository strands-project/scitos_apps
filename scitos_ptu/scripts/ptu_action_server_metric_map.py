#! /usr/bin/env python

import roslib; 
import rospy
import actionlib

from scitos_ptu.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import String
import math
import time
import flir_pantilt_d46.msg

class PTUServer:
  def __init__(self):
    rospy.init_node('ptu_actionserver_metric_map')
    self.pub = rospy.Publisher('/ptu/cmd', JointState)
    self.log_pub = rospy.Publisher('/ptu/log', String)
    self.state=rospy.Subscriber("/ptu/state", JointState, self.head_state_cb)

    self.server = actionlib.SimpleActionServer('ptu_pan_tilt_metric_map', PanTiltAction, self.execute, False)
    self.server.start()
    rospy.loginfo('Ptu action server started')
 
    self.reached_epsilon = math.radians(1.0)
    self.reached = False

    self.ptu_command = JointState()
    self.ptu_command.name=["pan", "tilt"]
    self.ptu_command.position=[0.0, 0.0]
    self.ptu_command.velocity=[1.0,1.0]

    self.feedback = scitos_ptu.msg.PanTiltFeedback()

    self.max_pan = 160.0 # degrees
    self.min_pan = -160.0 # degrees

    self.max_tilt = 30.0 # degrees
    self.min_tilt = -30.0 # degrees

    self.pan_tilt_timeout = 3 # seconds

    self.ptugoal = flir_pantilt_d46.msg.PtuGotoGoal()
    self.ptugoal.pan_vel = 21
    self.ptugoal.tilt_vel = 21

    self.client = actionlib.SimpleActionClient("SetPTUState", flir_pantilt_d46.msg.PtuGotoAction)
    self.client.wait_for_server()
    print 'Ptu client created'

  def head_state_cb(self,msg):
    if (math.fabs(msg.position[0] - self.ptu_command.position[0]) < self.reached_epsilon) and (math.fabs(msg.position[1] - self.ptu_command.position[1]) < self.reached_epsilon):
        self.reached = True
    else:
        self.reached = False

  def wait_for_ptu_motion(self,timeout):
    start_time = time.time()
    elapsed_time = 0
    while (not self.reached and elapsed_time < timeout):
        time.sleep(0.1)
        elapsed_time = time.time()-start_time

    if self.reached:
        return True
    else:
        return False

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
    self.ptu_command.position=[math.radians(-panstart),math.radians(-tiltstart)]
    self.ptugoal.pan = -panstart
    self.ptugoal.tilt = -tiltstart
    self.client.send_goal(self.ptugoal)
    self.client.wait_for_result()	    
    
    for i in range(panstart, panend, panstep):
       for j in range(tiltstart, tiltend, tiltstep):
    		self.ptugoal.pan = -i
		self.ptugoal.tilt =-j
    		self.client.send_goal(self.ptugoal)
    		self.client.wait_for_result()
		# keep this position for logging
          	self.log_pub.publish("start_position")
		time.sleep(2) # sleep for 2 seconds here
		self.log_pub.publish("end_position")

    self.ptugoal.pan = 0
    self.ptugoal.tilt =0
    self.client.send_goal(self.ptugoal)
    self.client.wait_for_result()  
    self.log_pub.publish("end_sweep")
    
    result = scitos_ptu.msg.PanTiltResult()
    result.ptu_pose = self.ptu_command
    self.server.set_succeeded(result)

if __name__ == '__main__':
  server = PTUServer()
  rospy.spin()
