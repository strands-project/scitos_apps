#! /usr/bin/env python

import roslib; 
import rospy
import actionlib

from scitos_ptu.msg import *

class PTUServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('ptu_pan_tilt', PanTiltAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print "moving the head"
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('ptu_server')
  server = PTUServer()
  rospy.spin()
