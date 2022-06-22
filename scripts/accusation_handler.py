#!/usr/bin/env python

"""
.. module:: accusation_handler
  :platform: Unix
  :synopsis: Python module that has the information
             about which is the right hypothesis

.. moduleauthor:: Iacopo Pietrasanta <iacopo.pietrasanta@gmail.com>

This module represents the "oracle", the entity that in the assignment 
specification it's presented as the entity that knows the correct hypothesis
(i.e. knows when, where and who commited the crime).
It's implementation is split in two services servers, thi one answers
if the correct hypo has been found or not when interrogated by the robot.

Subscribes to: 
              None
  
Publishes to: 
              None
  
Service : 
              /query_oracle to interrogate the oracle when a cadidate hypo
                            is being found
"""

import rospy
from er_assignment_1.srv import HypoID


def querySrvClbk(req):
    """
        Service callback for the query_oracle service server
    
        Args:
            req (HypoIDRequest): The ID of the candidate hypothesis (accusation)
            
    
        Returns:
            bool: True if the candidate ID is the same as the right ID (killer found), else False
    
        Raises:
            None
    
        Note:
            
    """
    if req.ID == rospy.get_param('True_id'):
        return True
    else:
        return False



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
            Initializes the the server.
    """

    rospy.init_node('accusation_handler')
    
    query_srv = rospy.Service('query_oracle', HypoID, querySrvClbk)
    
    rospy.spin()


if __name__ == '__main__':
    main()
