#!/usr/bin/env python

import rospy

import actionlib

from grid_plan.msg import *

class CommandActionExecutor(object):
  # create messages that are used to publish feedback/result
  _feedback = CommandFeedback()
  _result   = CommandResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, CommandAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    print goal
      
    if success:
      self._result.sequence = self._feedback.sequence
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('execute_server')
  FibonacciAction(rospy.get_name())
  rospy.spin()

