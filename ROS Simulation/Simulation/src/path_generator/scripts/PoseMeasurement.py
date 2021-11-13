#!/usr/bin/env python2
# General ROS and python libraries
import roslib
import rospy 
import roscpp
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped, Pose ,Twist
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Sim specific functions: 
from Simulation_Parameters import SwarmSimParams
from AgentModel import AgentModelNode

# =========================================================
# The following code is an implementation of the multiagent control protocol 
# presented in the paper:
# Robust Semi-Cooperative Multi-Agent Coordination In the Presence of Stochastic Disturbances 
# Authors: Kunal Garg  Dongkun Han Dimitra Panagou
# =========================================================

class PoseMeasNode(object):

    def __init__(self):

        # Calling script containing sim parameters 
        self.SimParam = SwarmSimParams()

        # Setting up all the variables to be used by the pose command generator 
        self.dt = 0.01

        # Calling the agent model function as the main driver for the mobile robot
        self.AgentModel= AgentModelNode()

        # Subscriber to the global position of the state of each of the robots. 
        self.SubPose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.swarm_pose_callback)
        
        # Subscriber to the odometry measurement for each of the agents 
        self.OdomRobot1 = rospy.Subscriber('/robot1/odom',Odometry, self.OdometryRobot1_Callback )
        self.OdomRobot2 = rospy.Subscriber('/robot2/odom',Odometry, self.OdometryRobot2_Callback )
        self.OdomRobot3 = rospy.Subscriber('/robot3/odom',Odometry, self.OdometryRobot3_Callback )
        self.OdomRobot4 = rospy.Subscriber('/robot4/odom',Odometry, self.OdometryRobot4_Callback )

        self.NumAgents = self.SimParam.Num_Agents[0]

        self.SetInitPose = 0 

        # Pose based on the Gazebo sim  pose  
        self.Pose_Gazebo = Pose() 
        self.PoseAct     = np.zeros((3,self.NumAgents))
        self.PoseMeas    = np.zeros((3,self.NumAgents))

        # Initialize odometry measurements 
        # Odom measurement robot1
        self.Robot1_linearMeas  = 0
        self.Robot1_AngularMeas = 0
        # Odom measurement robot2
        self.Robot2_linearMeas  = 0
        self.Robot2_AngularMeas = 0
        # Odom measurement robot3
        self.Robot3_linearMeas  = 0
        self.Robot3_AngularMeas = 0
        # Odom measurement robot4
        self.Robot4_linearMeas  = 0
        self.Robot4_AngularMeas = 0

        self.PoseOdomMeas = np.zeros((2,self.NumAgents))

        self.GlobalPoses = Pose()
        self.ListAgent= np.zeros((1,self.NumAgents))
        
    def OdometryRobot1_Callback(self,Odometry):
        self.Robot1_linearMeas  = Odometry.twist.twist.linear.x
        self.Robot1_AngularMeas = Odometry.twist.twist.angular.z

    def OdometryRobot2_Callback(self,Odometry):
        self.Robot2_linearMeas  = Odometry.twist.twist.linear.x
        self.Robot2_AngularMeas = Odometry.twist.twist.angular.z
    
    def OdometryRobot3_Callback(self,Odometry):
        self.Robot3_linearMeas  = Odometry.twist.twist.linear.x
        self.Robot3_AngularMeas = Odometry.twist.twist.angular.z
    
    def OdometryRobot4_Callback(self,Odometry):
        self.Robot4_linearMeas  = Odometry.twist.twist.linear.x
        self.Robot4_AngularMeas = Odometry.twist.twist.angular.z 
        
    def swarm_pose_callback(self, Odometry_msg):
        # gazebo sets up the order of the agent posese randomly so there will need to be some reorganizing done before going to the main simulation 
        # ['ground_plane', '2_robot', '3_robot', '4_robot', '1_robot']
        List = Odometry_msg.name
        # Get the list of IDs 
        Agent1_id = List.index("1_robot")
        Agent2_id = List.index("2_robot")
        Agent3_id = List.index("3_robot")
        Agent4_id = List.index("4_robot")
        
        self.GlobalPoses = Odometry_msg.pose
        self.ListAgent = np.array([Agent1_id,Agent2_id,Agent3_id,Agent4_id])     

    def PoseConfig(self,event):        
        # Check to see if the list of agents is 0
        if np.prod(self.ListAgent)!= 0: 
            # Initialize:
            for idx in range(0,self.SimParam.Num_Agents[0]): 
                # Set the agent ID to the stream number gazebo has set up for agent #1
                Agent_ID = self.ListAgent[idx]


                # Set up the variable holding the actual pose for each of the agents based on the actual pose in Gazebo
                self.Pose_Gazebo = self.GlobalPoses[Agent_ID] 
                
                
                # Lets define the pose for each agent which we will need for the path generation process:
                attitude = euler_from_quaternion((self.Pose_Gazebo.orientation.x,self.Pose_Gazebo.orientation.y,\
                                                self.Pose_Gazebo.orientation.z,self.Pose_Gazebo.orientation.w))                      


                self.PoseAct[0,idx] = self.Pose_Gazebo.position.x 
                self.PoseAct[1,idx] = self.Pose_Gazebo.position.y
                self.PoseAct[2,idx] = (attitude[2] + np.pi)%(2*np.pi)-np.pi

                # Defining the measured pose for the selected agent. 
                self.x_meas     = self.PoseAct[0,idx]  + self.SimParam.v_i[0, idx]
                self.y_meas     = self.PoseAct[1,idx]  + self.SimParam.v_i[1, idx]
                self.Theta_meas = self.PoseAct[2,idx] + self.SimParam.v_i[2,idx]

                self.PoseMeas[0,idx] = self.x_meas
                self.PoseMeas[1,idx] = self.y_meas
                self.PoseMeas[2,idx] = self.Theta_meas

                if idx == 0:
                    self.PoseOdomMeas[0,idx] = self.Robot1_linearMeas
                    self.PoseOdomMeas[1,idx] = self.Robot1_AngularMeas
                elif idx == 1:
                    self.PoseOdomMeas[0,idx] = self.Robot2_linearMeas
                    self.PoseOdomMeas[1,idx] = self.Robot2_AngularMeas
                elif idx == 2:
                    self.PoseOdomMeas[0,idx] = self.Robot3_linearMeas
                    self.PoseOdomMeas[1,idx] = self.Robot3_AngularMeas
                elif idx == 3:
                    self.PoseOdomMeas[0,idx] = self.Robot4_linearMeas
                    self.PoseOdomMeas[1,idx] = self.Robot4_AngularMeas
                else: 
                    print("Invalid index!")
            
            self.AgentModelCall()


    # Function used to call the agent model for command generation 
    def AgentModelCall(self):
        self.AgentModel.MobileAgentModel(self.PoseAct, self.PoseMeas,self.PoseOdomMeas, self.dt)


if __name__ == '__main__':
    rospy.init_node('Pose_Measurement', disable_signals = True)
    Agent = PoseMeasNode() 
    # sample time for calling the position and measurement initialize functions
    rospy.Timer(rospy.Duration(0.01), Agent.PoseConfig)
    rospy.spin()

