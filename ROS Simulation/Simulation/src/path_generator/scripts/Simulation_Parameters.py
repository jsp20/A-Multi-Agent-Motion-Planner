import roslib
import rospy 
import roscpp
import numpy as np
import random 
from numpy import linalg as LA
#================================================================
# Reference Paper used for parameter definition: 
# Robust Semi-Cooperative Multi-Agent Coordination In the Presence of Stochastic Disturbances 
# Authors: Kunal Garg  Dongkun Han Dimitra Panagou
# ================================================



# These constants are meant to be static which will be used by the various functions in this sim
class SwarmSimParams(object):

  def __init__(self):
    # Initialize the SwarmSimParams class 
    
    # Parameters related to model time
    self.StepSize  = 0.1 #seconds  

    # Parameters related to location of agents: 
    self.StartPose = np.array([[10, 10, 0.1],
                              [6.2, -6.2, 0.1],
                              [-10, 15, 0.1],
                              [-9, -5, 0.1]])


    self.GoalPose = np.array([[-10, -10],
                              [-6, 5],
                              [10, -15],
                              [9, 10]])


    # Parameters used to describe agent:
    self.Rho   = 0.4
    self.del_d = 1.42
    self.E_f   = 1.52

    # Number of agents in workspace: 
    self.Num_Agents = self.StartPose.shape
    # This is zero index [0] #row , [1] #col

    # Noise related to pose measurement:
    self.v_i  = np.random.uniform(size = (3,self.Num_Agents[0]))
    self.Pv_i = np.eye(3,dtype = int)*0.01

    # Parameters used for motion coordination:
    self.dm      = 2*self.Rho
    self.d_theta = np.sqrt(LA.norm(self.Pv_i)+1)
    self.d_d     = np.sqrt(2*(LA.norm(self.Pv_i)+1))
    self.d_prime = self.dm+ 2*self.d_d


    # Safety parameters for worst case neighbour 
    self.d_J      = ((2*self.d_d+self.dm)*np.sin(self.d_theta))/np.cos(self.d_theta)
    self.Rc       = 2*self.d_prime
    self.dc       = self.Rc
    self.dr       = (self.dc-self.d_prime)*random.uniform(0,1)+self.d_prime


    # Effect of wind 
    self.w_min    = 5.6*0.001
    self.w_max    = 7.25*0.001
    self.w_delta  = self.w_max-self.w_min*0.001


    # Make Ew and E equal for every agent 
    self.Ew       = np.random.uniform(size = 1)
    self.Ei       = np.random.uniform(size = 1)
    self.miu      = 10

    # Range of distance which is dm<de<dr
    self.d_e      = (self.dr-self.d_prime)*random.uniform(0,1)+self.d_prime

    # ================================================================================
    # Values used in the path generation script
    self.H_i      = np.eye(3,3)
    self.gamma    = np.array([[1,0],
                              [0,1],
                              [0,0]])
    
    self.Pw_i     =np.eye(2,2)*0.01
    # linear and angular velocity proportional gain values
    self.k_ui     = 0.7
    self.k_wi     = 0.5                      
if __name__ == '__main__':
  SwarmSimParams()