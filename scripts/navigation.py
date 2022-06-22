#!/usr/bin/env python

"""
.. module:: navigation
  :platform: Unix
  :synopsis: A "fake" navigation action with the correct interface
             

.. moduleauthor:: Iacopo Pietrasanta <iacopo.pietrasanta@gmail.com>

Python module that employs a "fake" ( next software versions
will ground it to more realistic applications ) action for
navigation and its interface. "fake" means it is simply a 
waiting function

Subscribes to: 
              None
  
Publishes to: 
              None
  
Service : 
              None
"""

import rospy
import actionlib
import math
from er_assignment_1.msg import *


class NavigationAction(object):
    """
      Navigation action server.

      Note:
        This navigation action has the complete interface to be expanded
        and updated for a much more realistic experiment with a real 
        environment. This implementation is simply a waiting process
        proportional to the euclidean distance between the simulated robot
        position and the goal destination.

      Args:
        Object:

      Attributes:
        as: action server
        pub: publisher to update the robot position
    """

    #_feedback = er_assignment_1.msg.NavigationFeedback()
    _result = er_assignment_1.msg.NavigationResult()
    _goal = er_assignment_1.msg.NavigationGoal()

    def __init__(self, name):
        self._action_name = name
        
        self._as = actionlib.SimpleActionServer(
    self._action_name,
    er_assignment_1.msg.NavigationAction,
    execute_cb=self.execute_cb,
     auto_start=False)
     
        self._as.start()

	
    def execute_cb(self, goal):
        """
            Performes the action
    
            Args:
                goal (NavigationGoal): desired position and actual position
            
    
            Returns:
                _result (NavigationResult): action result
    
            Raises:
                None
    
            Note:
                Computes the euclidean distance and wait proportionally,
                then updates the robot position.
        """
        r = rospy.Rate(1)
        success = True

        # initialising feedback fields
        # self._feedback.updated_x = position_.x
        # self._feedback.updated_y = position_.y

        if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False

        if success:
            dist = 2*(math.ceil(math.sqrt(pow(goal.actual_x - goal.desired_x, 2) + pow(goal.actual_y - goal.desired_y, 2))))
            rospy.loginfo('It will take %d time units', dist)
            rospy.sleep(dist)
            self._result.ok = True
            self._as.set_succeeded(self._result)       


def main():
    """
       Main function
    
       Args:
            None
            
    
       Returns:
            None
    
       Raises:
            None
    
       Note:
            Initializes action server
     """
	
    rospy.init_node('navigation')
    server = NavigationAction('navigation_action')
    
    rospy.spin()

if __name__ == '__main__':
	main()
    
