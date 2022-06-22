#!/usr/bin/env python

"""
.. module:: game_logic
  :platform: Unix
  :synopsis: Python module using smach to manage the game, it simulates 
             the different phases that the robot will traverse in order
             to complete the game

.. moduleauthor:: Iacopo Pietrasanta <iacopo.pietrasanta@gmail.com>

This node is the main logic of the system. It implements a state machine
using Smach libraries. It manages the simulation task of a task performed
by a robot playing cluedo, navigating in a minimally represent environment
( rooms ), gathering hints and asking an entity informed about the correct
conclusions ( the oracle ) regerding where , with what, and who committed 
the murder.
Starting from an initial state of INTEMPLE the logic will transition 
in state NAVIGATION to reach a new random room. Once reached ( state INROOM )
a client for the service roboCopActs is used for commanding the robot
to ask the oracle for a new hint ( a who or a what or a where ). 
The logic will then transition again to NAVIGATION, noving the robot to
another random room -> INROOM, to ask for a new hint. This will 
be repeated until the robot has collected enough hints to build a 
CONSISTENT hypothesis in its ontology. This interrogation of the 
ontology is performed by roboCop.py node. Once this conditon is 
met, a parameter retrieved by this node will signal this condition
to the logic, which will go in NAVIGATION state but this time, navigating
the robot to the state INTEMPLE. This state will also use roboCopActs 
client to command the robot to interrogate the oracle about the newly 
found candidate hypothesis.
If the cadidate hypothesis is the correct one, the game ends, otherwise
the logic resets the parameter and restart the process of NAVIGATION 
-> INROOM randomly as before, in order to build a new candidate hypo.

Subscribes to: 
              None
  
Publishes to: 
              None
  
Service : 
              None
"""

import rospy
import rosnode
import smach
import random
from er_assignment_1.srv import *
from er_assignment_1.msg import *


roboCop_client = None


class Navigate(smach.State):
    """
    State INROOM.

    Note:
        None

    Args:
        None

    Attributes:
        rooms(str list): list of available rooms
        rooms(str list): list of available rooms
    """
    def __init__(self):
        smach.State.__init__(self,
      			outcomes=['room_reached', 'temple_reached'])
                
        self.rooms=[
    'Ballroom',
    'Billiard_room',
    'Conservatory',
    'Dining_room',
    'Kitchen',
    'Hall',
    'Library',
    'Lounge',
    'Study']

    def execute(self, userdata):
        """
            The execution callback of the state
    
            Args:
                userdata : None
            
            Returns:
                smach.State: the next state to transition into
    
            Raises:
                None
    
            Note:
                Checks if the parameter representing the presence of a
                candidate hypo on not, and accondingly calls the action
                for a random room or for the temple.
                The next state in the logic is accordingly choosen using 
                the right transistion.
        """
        global roboCop_client
        
        rospy.wait_for_service('roboCopActs')
        v = roboCop_client("nav")
        
        if rospy.get_param("hasHypo") == True:
             return 'temple_reached'

        else:
             return 'room_reached'
        
        


class LookForClues(smach.State):
    """
    State INROOM.

    Note:
        None

    Args:
        None

    Attributes:
        None
    """

    def __init__(self):
        smach.State.__init__(self,
              		outcomes=['clue_found'])

    def execute(self, userdata):
        """
            The execution callback of the state
    
            Args:
                userdata : None
            
    
            Returns:
                smach.State: the next state to transition into
    
            Raises:
                None
    
            Note:
                It calls the servie SherlBotActs for commanding the robot 
                to ask the oracle for a new hint.
        """
        global roboCop_client
        
        rospy.wait_for_service('roboCopActs')
        v = roboCop_client("clue")
 
        return 'clue_found'



 
class QueryOracle(smach.State):
    """
    State INTEMPLE.

    Note:
        None

    Args:
        None

    Attributes:
        None
    """
    def __init__(self):
        smach.State.__init__(self,
              		outcomes=['killer_found','start_navigating'])

    def execute(self, userdata):
        """
            The execution callback of the state
    
            Args:
                userdata : None
            
    
            Returns:
                smach.State: the next state to transition into
    
            Raises:
                None
    
            Note:
                Besides the initialization of the state machine
                we transition in this state only when we have a 
                candidate hypo.
                If the interrogation will result positively, the game
                is concluded, otherwise the robot need to start 
                collecting hints again
        """
        global roboCop_client
        
        hasHypo = rospy.get_param("hasHypo")
        if hasHypo == True:
            rospy.wait_for_service('roboCopActs')
            w=roboCop_client("query")
        	
            if w.validation:
                return 'killer_found'
            else:
                rospy.set_param('hasHypo', False)
                return 'start_navigating'
        else:
            return 'start_navigating'





def main():
    """
        The main function
    
        Args:
            None
            
    
        Returns:
            None
    
        Raises:
            None
    
        Note:
            Initializes the smach state machine and its states and some variables
    """
    global roboCop_client
    
    rospy.init_node('game_logic')
    sm = smach.StateMachine(outcomes=['end_game'])
    roboCop_client = rospy.ServiceProxy('roboCopActs', CopMsg)
    
    with sm:
        smach.StateMachine.add('INTEMPLE', QueryOracle(), 
                               transitions={'killer_found':'end_game','start_navigating':'NAVIGATING'})
                               
        smach.StateMachine.add('NAVIGATING', Navigate(), 
                               transitions={'room_reached':'INROOM',
                               'temple_reached':'INTEMPLE'})
                               
        smach.StateMachine.add('INROOM', LookForClues(), 
                               transitions={'clue_found':'NAVIGATING'})
   

    outcome = sm.execute()
    rospy.signal_shutdown('NORMAL TERMINATION')
    
    rospy.spin()

if __name__ == '__main__':
    main()
    
