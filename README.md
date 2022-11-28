# Experimental Robotics Laboratory - first assignment package er_assignment_1

This package contains the source code implemented during the first assignment of the Experimental Robotics Laboratory, course of the first semester of the second year of Robotics Engineering Master Degree course of University of Genoa, Italy.

## Index

- Index:
  - [Required packages](#required-packages)
  - [Assignment request](#assignment-request)
  - [Hints and Locations](#hints-and-locations)
  - [Software Architecture and Functioning](#software-architecture-and-functioning)
  - [How to launch and additional documentation](#how-to-launch-and-additional-documentation)

## Required packages

The packages required are :

**smach_ros**
**ARMOR**

## Assignment request

The assignment required the implementation of a software architecture simulating a game of cluedo carried on by a *detective-like* robot. In this first version of the implementation, the robot model is not present, as well as a physically simulated model of the environment, which instead is simply represented as location in space. Therefore the navigation module is simply a dummy node, carrying movement operations simply *consuming* time in order to reach different locations. The different behaviours of the robot are triggered during different phases of the game, orchestrated by a finite state machine implemented using **smach** ros package. During this different phases the robot moves in the environment, and once it reaches locations (i.g. diffent rooms) it collects hints stored in the param server. Each hint has a particular structure that will be exposed in chapter (metti link), and are used to compose hypothesis on where, by who, and with which object the murder is been carried out. Once a particular condition for an hypothesis is reached ( Consistent, explained later ), the robot interrogates an *oracle* which knows the right answer, and once the right answer is collected the oracle gives positive feedback to the robot and the game is finished. The *knowledge* of the robot, represented by the collected hint inferring hypothesis, is stored and organized and inferred by **ARMOR** symbolic ontology.

## Hints and Locations

The hints are always composed as such :

- 1. An **ID**, specifying the hint information origin.
- 2. A *keyword* string representing the hint **category**, can be : *'who'*,*where* or *what*
- 3. A *keyword* string representing the **value** of the hint, according to its *category*

The **value** (3) can be one in these next lists, according to the selected category:

**where**:
*Ballroom*,
*Billiard_room*,
*Conservatory*,
*Dining_room*,
*Kitchen*,
*Hall*,
*Library*,
*Lounge*,
*Study*

**who**:
*Col.Mustard*
*Rev.Green*
*Prof.Plum*
*Mrs.Peacock*
*Mrs.White*
*Miss.Scarlett*
*Cpt.Brown*
*Srg.Gray*
*Dr.Orchid*

**what**:
*Tomahawk*
*Pencil*
*Hammer*
*TennisRacket*
*Candlestick*
*Revolver*
*LeadPipe*
*Wrench*
*Rope*

An hypothesis is only checked with the oracle if and only if is CONSISTENT, which for the ontology means is composed of 3 hints, one for each category. Otherwise is not consistent. The entire list of hints is loaded in the ros param server and is visible looking at the **param.yaml** file.

The various location reachable by the robot are these ones, with their location coordinates, used by navigation module to simulate movement of the robot:

Ballroom: [1.0,2.0]
Billiard_room: [1.0,3.0]
Conservatory: [1.0,4.0]
Dining_room: [0.5,4.0]
Hall: [-0.5,4.0]
Kitchen: [-1.0, 4.0]
Library: [-1.0,3.0]
Lounge: [-1.0,2.0]
Study: [-1.0,1.0]
Temple: [0.0,0.0]

## Software Architecture and Functioning

The architecture is composed of five nodes, besides the **ARMOR** node launched of start.

**roboCop.py**: This node represents the robot entity, which can perform 3 different activities:

- 1. Ask for an hint
- 2. Query the oracle about candidate CONSISTENT hypothesis
- 3. Call the action for navigating the ambient

This 3 behaviours are implemented by the service **/roboCopActs**, and trigger the call by clients to service **/ask_for_hint**(1) and **/query_oracle**(2) services, and the **/navigation_action**(3) action.

The node also implements the storing of new knowledge in the ontology.

**navigation.py**: This node implements the dummy action for navigating in the environment, it receives a call by a client specifying the position to be reached, computes the distance and proportionally takes time to reach it.

The action advertised is called **/navigation_action**.

**hint_handler.py**: This node is a simple server implementing the service **/ask_for_hint**. It receives an empty request, and responds providing an hint as response value.

**accusation_handler.py**: This node is a simple server implementing the service **/query_oracle**. It is called by a client sending a candidate ID for a CONSISTENT hypothesis, to which the node responds with a negative or positive response represented by a boolean, if the ID is the right one.

**game_logic.py**: This node implements the **smach** finite state machine representing the different phases of the game, represented by : being in a room, navigating, and being in the temple ( the room where the oracle is, reached once the robot has a CONSISTENT hypothesis to check ).

Here are the diagrams representing the overall architecture:
![alt comnponent](https://github.com/HolyStone95/er_assignment_1/blob/master/media/Component_UML.png)
The finite state machine:
![alt fsm](https://github.com/HolyStone95/er_assignment_1/blob/master/media/FSM_UML.png)
The most important execution flow of the program ( The navigation phase is the only one not present, cause it simply consist of a *waiting* service ):
![alt sequence](https://github.com/HolyStone95/er_assignment_1/blob/master/media/Sequence_INTEMPLE_UML.png)

## How to launch and additional documentation

To launch the simulation, build the workspace and then launch:
```
 roslaunch er_assignment_1 sim.launch
```
Additional documentation regarding the project can be found in the **_build** folder, particularly opening the file index.html in the **html** folder
