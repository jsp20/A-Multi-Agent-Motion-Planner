Date of creation: July 2020
Supervisor: Hugh Liu - UTIAS
Class: MIE Y8888 - Research Project 
Author: John Sebastian Pineros 

Order for running the multi-agent simulation: 
- Open the "Simulation_Parameters.m" file which holds the common variables for all the 
functions used in the swarm simulation. 
- For editing the number of agents change the variable q_goal = [ x_g y_g ]' to define the 
goal location and q_O = [x_0 y_0 theta_0 ]' for defining the starting positions. 

For plotting data once the simulation has finished: 
- Open the "Plotting_Script.m" file which has preset variables which are saved to the workspace for plotting 
once the simulation is finished. 
- By default the simulation has defined saved variables from 1-7 agents with 1-4 as the variables 
uncommented and agents 5-7 commented out. If you'd like to set up a script for plotting 7 agents uncomment the 
necessary variables an ensure the legends are modified for the 7 agent case. 

For plotting saved data from ROS. 
- The ROS simulation has been set up to save two variables from the gazebo simulation:
The actual pose of the TurtleBot2 in space and the computation time for computing the pose command 
for all 4 agents.
