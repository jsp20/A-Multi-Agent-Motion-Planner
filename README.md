# A-Multi-Agent-Motion-Planner
University of Toronto MEng project focused on multi-agent motion planning in the presence of stochastic noise

Date of creation: July 2020
Supervisor: Hugh Liu - UTIAS
Class: MIE Y8888 - Research Project 
Author: John Sebastian Pineros 

The following repository is for the MEng Research project conducted at the University of Toronto as part of the CARRE Emphasis in Aerial Robotics. It focuses on a multi-agent motion planning protocol which can deal with stochastic noise and generate an obstacle free path. 

The repo is divided in two section, the first is focused in the simulation implemented in MATLAB script to simulate the motion planning protocol. MATLAB was also used to calculate the maximum number of agents and the computation time increase related to the number of agents. 

The second subfolder of this repo is the ROS1 implementation of this code. This implementation focuses on using 4 turtlebot2 agents and simulates the obstacle free trajectory using Gazebo and ROS kinetic in Ubuntu 16.04. 

You can find a copy of the technical report by going to https://johnspineros.ca/ and checking out my welcome blog.

For a demonstration of the simulation results in ROS: 

https://youtu.be/UqKj3HIz-3Y

The following to animations show the result of the multiagent protocol running for 7 and 4 agents. 

Example results for 7 agents based on MATLAB implementation  
![7_Agent_Sim](https://user-images.githubusercontent.com/47089025/141601732-feff5af2-716f-48b4-aed5-6e07dc569a27.gif)

Example results for 4 agents based on MATLAB implementation

![4_Agent_Sim](https://user-images.githubusercontent.com/47089025/141601756-ac364dc2-40a9-4463-b30b-6f7724986683.gif)
