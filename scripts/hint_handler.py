#!/usr/bin/env python

"""
.. module:: hint_handler
  :platform: Unix
  :synopsis: Python module that has the information about hints

.. moduleauthor:: Iacopo Pietrasanta <iacopo.pietrasanta@gmail.com>

This module represents the "oracle", the entity that in the assignment 
specification it's presented as the entity that knows the correct hypothesis
(i.e. knows when, where and who commited the crime),and has a list of all
possible hints identifiers.

It's implementation is split in two services servers, this one provides
a new hint to the robot when requested.

Subscribes to: 
              None
  
Publishes to: 
              None
  
Service : 
              /ask_for_hint to ask the oracle for a new hint
"""

import rospy
import random
from er_assignment_1.srv import Hint, HintResponse


IDs = None
"""List of strings to store possible hints identifiers
"""

def hintSrvClbk(req):
    """
        Service callback for the ask_for_hint service server
    
        Args:
            req (HintRequest): Empty request
            
        Returns:
            res (HintResponse): a list of 3 strings representing a particular hint
    
        Raises:
            None
    
        Note:
            Every time a particular hint is sent back, it's removed
            from the list of possible hints, and its value is removed
            on the parameter server to avoid repetitions of alredy 
            seen information.
    """
    global IDs
    
    nextID = random.choice(IDs)
    
    res = HintResponse()
    res.hint = rospy.get_param(nextID)
    res.hint.append(nextID[0:3])
    
    IDs.remove(nextID)
    rospy.delete_param(nextID)

    return res




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
            Initializes possible hints identifiers.
            Initializes the server.
    """
    global IDs

    rospy.init_node('hint_provider')
    
    IDs = ["ID1_1", "ID1_2", "ID1_3", "ID1_4",
           "ID2_1", "ID2_2", "ID2_3",
           "ID3_1", "ID3_2", "ID3_3", "ID3_4", 
           "ID4_1", "ID4_2", "ID4_3", "ID4_4", 
           "ID5_1", "ID5_2", "ID5_3",
           "ID6_1", "ID6_2", "ID6_3", 
           "ID7_1", "ID7_2", "ID7_3", "ID7_4",
           "ID8_1", "ID8_2", "ID8_3", "ID8_4",
           "ID9_1", "ID9_2", "ID9_3"]
    
    hint_srv = rospy.Service('ask_for_hint', Hint, hintSrvClbk)
    
    rospy.spin()


if __name__ == '__main__':
    main()
