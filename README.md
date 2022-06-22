# Research Track I - first assignment package bot_simulation

This package contains the source code of the two nodes necessary in order to complete the first assignment of Research Track 1 course of Robotics Engineering Laurea Magistrale, academic year 2020/2021. This work is a completely original work of Iacopo Pietrasanta.

## How to launch and additional documentation

In order to launch the simulation the packages **stage_ros** and **assignment1** needs to be installed. In the package assignment1 the file exercise.world needs to be present in the world folder.
If these prerequisites are met , the simulation can be launched with a launch file via command line:
```
 roslaunch bot_simulation assignment1.launch
```
Additional documentation regarding the project can be found in the docs folder, particularly opening the file index.html in the **html** folder

## Behaviour

Once the simulation is launched the robot will repeat in a loop these next steps:

- 1. The robot asks for a random target, with both coordinates in the interval (-6.0, 6.0).
- 2. The robot reaches the target.
- 3. Go to step 1.

## Architecture

This package is composed of two nodes contained in the **src** folder:

- **controller.cpp** implements a subscriber to /odom to retrive robot pose, a publisher to /cmd_vel to set the robot velocity and a client to the service /goal in order to request a new random target in a range specified by the request.

- **targeting_server.cpp** simply implementes a server for the service /goal and a function **randMToN** that calculate the actual random values once the request has been received.

The service file for /goal is contained in the folder **srv** of the package, precisely it's called **Goal.srv**:

request:

float32 min

float32 max

response:

float32 x

float32 y

where min and max defines the range(request), and x and y are the new target position(response)

## Rqt_Graph

![alt rqt_graph](https://github.com/HolyStone95/RT1_Assignment1/blob/master/rosgraph.png)



