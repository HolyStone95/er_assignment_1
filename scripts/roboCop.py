#!/usr/bin/env python

"""
.. module:: roboCop
  :platform: Unix
  :synopsis: Python module representing the robot and its knowledge

.. moduleauthor:: Iacopo Pietrasnta <iacopo.pietrasanta@gmail.com>

This node represents the robot entity, which can perform 3
diffent activities:

- Ask for an hint
- Query the oracle about candidate CONSISTENT hypo
- Call the action for navigating the ambient

The distinction between the first two aforementioned activities
is managed by a server for service roboCopActs that, based on the value 
it retrieves from the request, calls one of these two services, 
through clients:

- ask_for_hint
- query_oracle

The first returns an hint which is then stored in the ontology
by this node, and the second one returns True if the interrogation
of the oracle about a CONSISTENT hypo has positive outcome,
otherwise False.
In the third case we call the function called nav_client that will
populate the action message and call the action server.
The full annotations for the two services are in their provider files,
query_oracle.py, hint_oracle.py and navigation.py.
The CopMsgResponse for roboCopActs service response is populated 
accordingly, and sent back to client in the main logic of the 
system, to continue with the game of Cluedo.

A CONSISTENT hypo is gathered by a function that each time a new hint
is stored, interrogates the ontology. If a CONSISTENT hypothesis
is found, it is stored into a queue for managing the priority
of discoveries on cadidates hypos, and a parameter in ros server
is set to True, changing the transistions of states in robot_controller.py
i.e. the system logic, representing the knowledge of the robot of 
having found a candidate hypo for interrogating the oracle.

Subscribes to: 
              None
  
Publishes to: 
              None
  
Service : 
              /roboCopActs to perform an action
"""

import queue as qu
import rospy as rp
import random
import actionlib
from armor_api.armor_client import ArmorClient
from er_assignment_1.srv import CopMsg, Hint, HypoID, CopMsgResponse, HypoIDResponse, HintRequest
from er_assignment_1.srv import *
from er_assignment_1.msg import *


actual_loc = "Temple"
""" string: stores the actual location of the robot 
"""
hint_client = None
""" global variable to store client of service ask_for_hint.
"""
query_client = None
""" global variable to store client of service query_oracle.
"""
consistent = None
""" global variable to store consistent hypothesis.
"""
armor_client = None
""" global variable to store client of armor service.
"""
checked=[]
"""list: empty list for storing already checked consistent hypothesis
"""
rooms=[
    'Ballroom',
    'Billiard_room',
    'Conservatory',
    'Dining_room',
    'Kitchen',
    'Hall',
    'Library',
    'Lounge',
    'Study']
"""list: list for storing the rooms names used for navigation
"""





def srvClbk(req):
    """
        Callback function for SherlBotActs service .
    
        Args:
            req (CopMsgRequest): composed of a boolean used for distinguishing different requests
            
    
        Returns:
            res(CopMsgResponse): depending on the request's values, it is populated accordingly
        
        Raises:
            None
    
        Note:
            If req is True, through query_client calls for query_oracle service.
            If req is False, through hint_client calls for ask_for_hint service.
    """
	
    global hint_client, query_client, consistent, armor_client, checked, rooms, actual_loc
    res = CopMsgResponse()
    
    
    if req.command == "clue":
        rp.wait_for_service('ask_for_hint')
        hint = hint_client()
        
        gatherHint(hint)
        checkConsistency()
        res.validation = True
		
    elif req.command == "query":
        rp.wait_for_service('query_oracle')
        
        iD_raw = consistent.get()
        where = clean_str(armor_client.query.objectprop_b2_ind('where', iD_raw))
        what = clean_str(armor_client.query.objectprop_b2_ind('what', iD_raw))
        who = clean_str(armor_client.query.objectprop_b2_ind('who', iD_raw))
        rp.loginfo("--I think the murder was committed in the %s, with the %s, and the killer is %s.--" % (where, what, who))
        
        iD = iD_raw.replace('HP','ID')
        valid = query_client(iD)
        
        if valid.validation == False: 
            rp.loginfo("--No, you're wrong. Keep looking you dumb tincan of a robot!!!--")
            checked.append(iD_raw)
        elif valid.validation== True:
            rp.loginfo("--Yes!!! Finally you got him. Maybe in a thousand years you'll be a good detective.--")
            
        res.validation = valid.validation
        
    elif req.command == "nav":
        hasHypo = rp.get_param("hasHypo")
        actual_loc = rp.get_param('actual_loc')
        
        print("\n")
        
        if hasHypo == True:
            rp.loginfo('Moving from the %s to the Temple ', actual_loc)
            result = nav_client(rp.get_param('Temple'))
        else:
            while True:
                nextRoom = random.choice(rooms)
                if nextRoom != actual_loc:
                   break
                   
            rp.loginfo('Moving from the %s to the %s ', actual_loc, nextRoom)
            nextRoomLoc = rp.get_param(nextRoom)
            result = nav_client(nextRoomLoc)
            rp.set_param('actual_loc',nextRoom)
            res.validation = True
            
    return res





def nav_client(desired):
        """
            Function for calling the navigation action
    
            Args:
                desired (int list): list of two elements represesenting
                desired planar position
            
            Returns:
                client.get_result() (NavigationResult): action result
    
            Raises:
                None
    
            Note:
                Populates a NavigationGoal msg with the desired position
                and make the call with a client.
        """
        global actual_loc
        
        client = actionlib.SimpleActionClient(
    'navigation_action',
     er_assignment_1.msg.NavigationAction)
        client.wait_for_server()
        goal = er_assignment_1.msg.NavigationGoal()
        goal.desired_x = desired[0]
        goal.desired_y = desired[1]
        goal.actual_x = rp.get_param(actual_loc)[0]
        goal.actual_y = rp.get_param(actual_loc)[1]
        client.send_goal(goal)
        client.wait_for_result()

        return client.get_result()





def gatherHint(hint):
    """
        saveHint function is exploited to save hint within the ontology .
    
        Args:
            hint (HintResponse): the new hint receiced from the oracle, to be stored in the ontology
            
    
        Returns:
            None
    
        Raises:
            None
    
        Note:
            Based on the keyword identifier of the type of hint, it is saved in the ontology
            accordingly
    """
    global armor_client
    
    if hint.hint[0] == 'where':
            armor_client.manipulation.add_ind_to_class(hint.hint[1], 'PLACE')
            armor_client.manipulation.add_ind_to_class('HP'+ hint.hint[2][2], 'HYPOTHESIS')
            armor_client.manipulation.disj_inds_of_class('PLACE')
            armor_client.manipulation.add_objectprop_to_ind('where', 'HP'+ hint.hint[2][2], hint.hint[1])
    if hint.hint[0] == 'who':
            armor_client.manipulation.add_ind_to_class(hint.hint[1], 'PERSON')
            armor_client.manipulation.add_ind_to_class('HP'+ hint.hint[2][2], 'HYPOTHESIS')
            armor_client.manipulation.disj_inds_of_class('PERSON')
            armor_client.manipulation.add_objectprop_to_ind('who' , 'HP'+ hint.hint[2][2], hint.hint[1])
    if hint.hint[0] == 'what':
            armor_client.manipulation.add_ind_to_class(hint.hint[1], 'WEAPON')
            armor_client.manipulation.add_ind_to_class('HP'+ hint.hint[2][2], 'HYPOTHESIS')
            armor_client.manipulation.disj_inds_of_class('WEAPON')
            armor_client.manipulation.add_objectprop_to_ind('what', 'HP'+ hint.hint[2][2] , hint.hint[1])
            
    rp.loginfo('I found an hint!!! : %s, with %s', hint.hint[1], hint.hint[2])
    armor_client.utils.save_ref_with_inferences('/root/ros_ws/src/er_assignment_1/inferred.owl')






def checkConsistency():
    """
        Function that once the robot collects an hint,
        interrogates the ontology for checking the hypothesis 
        consistency
    
        Args:
            None
            
    
        Returns:
            None
    
        Raises:
            None
    
        Note:
            CONSISTENT hypothesis are COMPLETED but NOT INCONSISTENT.
            This function keeps also track of already checked CONSISTENT
            hypothesis
    """
    global armor_client, consistent, checked
    
    compl = armor_client.query.ind_b2_class('COMPLETED')
    incon = armor_client.query.ind_b2_class('INCONSISTENT')
	
    compl=clean_str(compl)
           
    incon=clean_str(incon)
	
    for i in range(0,len(checked)):
        compl.remove(checked[i])
		
    for i in range(0,len(compl)):
        try:  
            index = incon.index(compl[i])
        except ValueError:
            rp.set_param('hasHypo', True)
            consistent.put(compl[i])
            rp.loginfo('Found a new consisten Hypothesis : %s' , compl[i])


def clean_str(strg):
    
    for i in range(0,len(strg)):
        start = strg[i].find('#')
        end = strg[i].find('>')
        strg[i] = strg[i][start+1:end]
        
    return strg

    

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
            It initializes two clients ("ask_for_hint", "query_oracle" services)
            and the armor client to work directly on the ontology.
            It initializes a server for service "ShelBotActs"
            Moreover, a queue has been defined for storing CONSISTENT hypo.
            
    """
    global armor_client, hint_client, query_client, consistent, actual_loc
    
    rp.init_node('roboCop')
    hint_client = rp.ServiceProxy('ask_for_hint', Hint)
    query_client = rp.ServiceProxy('query_oracle', HypoID)
    consistent = qu.Queue()
    armor_client = ArmorClient('tutorial', 'ontoTest')
    armor_client.utils.load_ref_from_file('/root/ros_ws/src/er_assignment_1/cluedo_ontology(1).owl', 'http://www.emarolab.it/cluedo-ontology', False, 'PELLET', False, True)
    armor_client.utils.set_log_to_file(True,'/root/ros_ws/src/er_assignment_1/logs/log.log')
    srv = rp.Service('roboCopActs', CopMsg, srvClbk)
    
    rp.spin()





if __name__ == '__main__':
    main()
