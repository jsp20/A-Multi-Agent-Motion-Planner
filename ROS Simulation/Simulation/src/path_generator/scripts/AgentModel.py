#!/usr/bin/env python2
# General ROS and python libraries
import roslib
import rospy 
import roscpp
import math
import numpy as np
from numpy import linalg as LA
from geometry_msgs.msg import TransformStamped, Pose ,Twist


#================================================================
# The following code is an implementation of the multiagent control protocol 
# presented in the paper:
# Robust Semi-Cooperative Multi-Agent Coordination In the Presence of Stochastic Disturbances 
# Authors: Kunal Garg  Dongkun Han Dimitra Panagou
# ================================================
# Sim specific functions
from Simulation_Parameters import SwarmSimParams

class AgentModelNode(object):

    def __init__(self):

        

        # setup publishers for all 4 robots: 
        self.PubRobot1 = rospy.Publisher('/robot1/mobile_base/commands/velocity',Twist, queue_size= 32)
        self.PubRobot2 = rospy.Publisher('/robot2/mobile_base/commands/velocity',Twist, queue_size= 32)
        self.PubRobot3 = rospy.Publisher('/robot3/mobile_base/commands/velocity',Twist, queue_size= 32)
        self.PubRobot4 = rospy.Publisher('/robot4/mobile_base/commands/velocity',Twist, queue_size= 32)
        
        
        # ======================Global variables used for the mobile agent function======================
        self.initialize = 0

        # Calling script containing sim parameters 
        self.SimParam = SwarmSimParams()
        self.NumAgents = self.SimParam.Num_Agents[0]

        self.publishIndex = 0        
        self.w_bar     = np.zeros((2,1))
        self.ui        = np.zeros((1,self.NumAgents))
        self.ui_n      = np.ones((1,self.NumAgents))*0.1
        self.ui_c      = 0
        
        # initializing pose rate commands
        self.PoseRate = np.ones((2,self.NumAgents))*0.001

        # Predicted Measured pose
        self.PoseMeasPred = np.zeros((3,self.NumAgents))

        # To be used in trajectory generation
        self.PreviousPose= np.zeros((3,self.NumAgents))

        # ====================== Global variables used in observer model ====================== 
        #initializing EKF observer model 
        self.PosePred = np.zeros((3,self.NumAgents))

        self.Ai           = np.zeros((3,3))
        self.List         = np.zeros(self.NumAgents)
        self.Ki           = []
        self.Pi_dot       = []
        self.Pi           = []

        for i in range(self.SimParam.Num_Agents[0]):
            self.Ki.append(np.eye(3,3)*0.1)
            self.Pi_dot.append(np.eye(3,3)*0.1)
            self.Pi.append(np.zeros((3,3)))  

        self.ThetaError = 0
        # ====================== Global variables used in reference field generation ====================== 
        
        # variables used in the potential field calculation 
        self.Fi_x = 0
        self.Fi_y = 0
        self.Fi   = np.zeros((2,1))

        # Partial derivatives for attractive field
        self.dFi_Ax_dx = 0
        self.dFi_Ax_dy = 0
        self.dFi_Ay_dx = 0
        self.dFi_Ay_dy = 0

        # Partial derivatives for repulsive field  
        self.dFi_R_ijx_dx = np.zeros((1,self.NumAgents))
        self.dFi_R_ijx_dy = np.zeros((1,self.NumAgents))
        self.dFi_R_ijy_dx = np.zeros((1,self.NumAgents))
        self.dFi_R_ijy_dy = np.zeros((1,self.NumAgents))
  
        self.sigma_ij     = np.zeros((1,self.NumAgents))

        # ====================== Global variables used in reference field variables used in agent model ======================

        # x-y components for normalized reference field function Fi
        self.Fi_pnx = 0
        self.Fi_pny = 0 

        # Orientation of perturbed reference field for each agent 
        self.psi_p = 0

        # Partial derivatives of the normalized perturbed reference field Fi_n
        self.dFi_pnx_dx = 0
        self.dFi_pnx_dy = 0
        self.dFi_pny_dx = 0
        self.dFi_pny_dy = 0
        

        # List of agents at goal location 
        self.AgentsAtGoal = np.zeros((1,self.NumAgents))
               
        
   
    def MobileAgentModel(self,PoseAct, PoseMeas,PoseOdomMeas,dt):
        # ==================Documentation================== 
        # Input:
        # PoseAct      - The ground truth location of the agent in space. 
        # PoseMeas     - IF state measurement of all the robots in space.
        # PoseOdomMeas - BF linear and angular velocity measurements.  
        # dt           - Time step duration.    

        # Defining the average wind noise based on the number of agents and the upper and lower bound of wind disturbance
        # Wind parameters to be used in main sim  
        # This is defined as the effect of wind disturbance on each agent in the x and y direction
        # w_bar is a [2xm] matrix for [x,y]' disturbance for m agents
        self.w_bar     = ((self.SimParam.w_max-self.SimParam.w_min)*np.random.uniform(size = (2,self.SimParam.Num_Agents[0]))+self.SimParam.w_min)*dt
       
        
        # Initialize the predicted pose and predicted measured pose at the start of the simulation
        if self.initialize == 0:
            self.PosePred = PoseAct
            self.PoseMeasPred = PoseMeas
            self.initialize += 1

        if np.prod(self.AgentsAtGoal) == 0:
            time_Start = rospy.get_rostime()
            Start = time_Start.nsecs*pow(10,-9)

            # It is assumed that the current agent has access to the current state of all other agents in the workspace 
            for Agent_ID in range(0,self.SimParam.Num_Agents[0]):
                
                # for plotting purposes
                file_name = file('ROS_Pose_Log.txt','a')
                np.savetxt(file_name, PoseAct[:,Agent_ID])
                file_name.close()
                
                if self.AgentsAtGoal[0,Agent_ID] == 1:
                    Agent = Agent_ID+1
                    # print("Agent ", Agent ,"Arrived at the goal location!")
                    continue


                # resetting variables at the start of the callback function 
                Jk_pred   = np.zeros((1,self.NumAgents))
                rk        = np.zeros((2,self.NumAgents))
                ri_pred   = np.zeros((2,self.NumAgents))
                xi_k_pred = []
                yi_k_pred = []
                ri_k_pred = []
                di_k_pred   = [] 
                Neigh_ID  = []
                ui_k      = np.zeros((1,self.NumAgents))
                L         = []
                Omegai    = 0

                # Check to see if the agent has arrived to the goal location:                
                GoalDistCheck = np.array([PoseAct[0,Agent_ID]-self.SimParam.GoalPose[Agent_ID,0],PoseAct[1,Agent_ID]-self.SimParam.GoalPose[Agent_ID,1]])
                
                if LA.norm(GoalDistCheck) < 0.7:
                    Agent = Agent_ID+1
                    self.PoseRate[0, Agent_ID] = 0.01
                    self.PoseRate[1, Agent_ID] = 0.01
                    self.AgentsAtGoal[0,Agent_ID] = 1
                    continue
                
                

                PoseNeigh      = np.zeros((2,self.NumAgents))         

                # Checking which is the nearest neighbour for agent i   
                for k in range (0,self.SimParam.Num_Agents[0]):
                    if k !=  Agent_ID:
                        PoseNeigh[0,k] = PoseAct[0,k]
                        PoseNeigh[1,k] = PoseAct[1,k]

                        
                        Agent_Dist = np.array([PoseNeigh[0,k]-PoseAct[0,Agent_ID],
                                                PoseNeigh[1,k]-PoseAct[1,Agent_ID]])  

                        dist = LA.norm(Agent_Dist)
                        
                        if dist < self.SimParam.Rc:
                            rk[0,k]   = PoseNeigh[0,k]
                            rk[1,k]   = PoseNeigh[1,k]
                            Neigh_ID.append(k)

                # Calling the observer model
                self.SystemObserver(PoseAct,PoseMeas, Agent_ID, dt)

                # Once the predicted states have been calculated by the system observer now calculate the reference field forces:
                self.RefField(PoseAct, rk, self.ui_n, Agent_ID,Neigh_ID) 
                
                # using the list of critical neighbours lets check which ones match the first condition for u_i given in equation 9  
                # check if any agent within the sensing radius matches condition 1 for the
                # piecewise function, if not then the second condition needs to be checked.
                # calcuation of the linear velocity for agent i 
                if Neigh_ID  != []:
                    # For a non empty list lets extract the neighbours to check         
                    for index in range(0,len(Neigh_ID)):    
                        j = Neigh_ID[index]   
                        xi_k_pred= [(self.PosePred[0,Agent_ID]-self.PosePred[0,j])] 
                        yi_k_pred =[(self.PosePred[1,Agent_ID]-self.PosePred[1,j])]  
                        ri_k_pred = np.array([xi_k_pred,yi_k_pred])
                        di_k_pred.append(LA.norm([ri_k_pred]))                   
                        
                        if self.SimParam.d_prime <= di_k_pred[index] and di_k_pred[index]<= self.SimParam.d_e:
                            #  this is a list of all the agents that are within the safety zone of agent i. 
                            L.append(j)

                GoalPoseError = np.array([ self.PosePred[0,Agent_ID]-self.SimParam.GoalPose[Agent_ID,0], self.PosePred[1,Agent_ID]-self.SimParam.GoalPose[Agent_ID,1] ])
                self.ui_n[0,Agent_ID] = self.SimParam.k_ui*np.tanh(LA.norm(GoalPoseError))

                # Checking if any of the agents are within condition 1 for velocity
                rki = np.zeros((1,2))
                if L != [] and di_k_pred!=[]:
                    for index in range(0,len(L)):
                        j = L[index]  

                        ni_pred = np.array([np.cos(self.PosePred[2,Agent_ID]) , np.sin(self.PosePred[2,Agent_ID])])
                        ni_pred = ni_pred.reshape(-1,1)
                        nk_pred = np.array([np.cos(self.PosePred[2,j]), np.sin(self.PosePred[2,j])])
                        
                        xi_k_pred = self.PosePred[0,Agent_ID]-self.PosePred[0,j] 
                        yi_k_pred = self.PosePred[1,Agent_ID]-self.PosePred[1,j]      
                                            
                        rki[0,0] = xi_k_pred
                        rki[0,1] = yi_k_pred                        
                                            
                        
                        uis_k =  (self.SimParam.Ei*PoseOdomMeas[0,Agent_ID] +(self.SimParam.d_prime+self.SimParam.Ew)*self.SimParam.w_delta)*np.asscalar(np.dot(rki,ni_pred))/np.asscalar((np.dot(rki,nk_pred)))       
                        ui_k[0,j] = self.ui_c*(di_k_pred[index]-self.SimParam.d_prime)/(self.SimParam.d_e-self.SimParam.d_prime)+uis_k*(self.SimParam.d_e-di_k_pred[index])/(self.SimParam.d_e-self.SimParam.d_prime)                                     
                    
                    # This is where we estimate the smallest velocity of agent i if there are any agents which meet condition 1 of the linear velocity control law    
                    self.ui[0,Agent_ID] = -1/self.SimParam.miu*np.log(np.sum(np.exp(-self.SimParam.miu*ui_k),axis =1))
                    # If there are no agents which meet condition 1 then we default to setting the linear velocity equal to the nominal velocity u_ic.
                else:
                    self.ui[0,Agent_ID] = self.ui_c 


                # Saturation limits for the linear velocity of the agents:
                if self.ui[0,Agent_ID] > 0.7: # m/s
                    self.ui[0,Agent_ID] = 0.7
                elif np.absolute(self.ui[0,Agent_ID]) < 0.1: # m/s
                    self.ui[0,Agent_ID] = 0.1
                elif self.ui[0,Agent_ID] < -0.7:
                    self.ui[0,Agent_ID] = -0.7 # m/s
                    pass

                # Setting up the linear and angular velocities
                # Compute si_dot
                psi_dot_p = 0
                psi_dot_p = ((self.dFi_pny_dx*np.cos(self.PosePred[2,Agent_ID])+self.dFi_pny_dy*np.sin(self.PosePred[2,Agent_ID]))*self.Fi_pnx-(self.dFi_pnx_dx*np.cos(self.PosePred[2,Agent_ID])+(self.dFi_pnx_dy*np.sin(self.PosePred[2,Agent_ID])))*self.Fi_pny)* self.ui[0,Agent_ID]

                # Angular velocity
                # Eq13
                
                self.ThetaError = self.PosePred[2,Agent_ID]-self.psi_p
                # wrap error between 0 and pi
                self.ThetaError = (self.ThetaError + np.pi) % (2 * np.pi) - np.pi
                # ThetaError = (ThetaError + np.pi) % (2 * np.pi)
                Omegai = -self.SimParam.k_wi*(self.ThetaError)+psi_dot_p*0.1
                # Omegai = -self.SimParam.k_wi*(self.ThetaError)


                # Saturation rate for mobile base angular velocity
                # For the kobuki base the absolute rate limit angularly is 180 deg/s
                if Omegai >= np.pi: 
                    Omegai = np.pi
                elif Omegai < -np.pi:
                    Omegai = -np.pi  
                    pass


                # The commanded linear and angular velocities based on the ref field logic
                self.PoseRate[0, Agent_ID] = self.ui[0,Agent_ID]
                self.PoseRate[1, Agent_ID] = Omegai
    
            time_Finish = rospy.get_rostime()
            Finish = time_Finish.nsecs*pow(10,-9)
            CompTime = np.array ([Finish-Start])

            # for plotting purposes
            file_name = file('CompTime.txt','a')
            np.savetxt(file_name, CompTime)
            file_name.close()
        
        # Only write motion commands when not all agents are in the goal location 
        if np.prod(self.AgentsAtGoal) == 0:
            for Agent_ID in range(0,self.SimParam.Num_Agents[0]):
                # Setup a timer to publish position periodically
                self.publishIndex = Agent_ID
                # call the pose publish function for one step             
                publisher_timer =  self.PoseCommandGeneration()
        elif np.prod(self.AgentsAtGoal) == 1:
            print("All Agents have arrived!")
      
    def PoseCommandGeneration(self):
            poseRateCMD = Twist()

            if self.publishIndex == 0:
                poseRateCMD.linear.x   = self.PoseRate[0,self.publishIndex]
                poseRateCMD.angular.z  = self.PoseRate[1,self.publishIndex]
                self.PubRobot1.publish(poseRateCMD)

            elif self.publishIndex == 1:
                poseRateCMD.linear.x   = self.PoseRate[0,self.publishIndex]
                poseRateCMD.angular.z  = self.PoseRate[1,self.publishIndex]
                self.PubRobot2.publish(poseRateCMD)

            elif self.publishIndex == 2:
                poseRateCMD.linear.x   = self.PoseRate[0,self.publishIndex]
                poseRateCMD.angular.z  = self.PoseRate[1,self.publishIndex]
                self.PubRobot3.publish(poseRateCMD)

            elif self.publishIndex == 3:
                poseRateCMD.linear.x   = self.PoseRate[0,self.publishIndex]
                poseRateCMD.angular.z  = self.PoseRate[1,self.publishIndex]
                self.PubRobot4.publish(poseRateCMD)

            else: 
                print("Incorrect Agent Number!!!")

            rospy.sleep(0.01)

    def SystemObserver(self, PoseAct,PoseMeas,Agent_ID,dt):
        # This is the observer model which will predict the pose of the agent based on the previous predicted pose & measurements 
        # Output of the system observer will be used to compute the linear and angular velocities 

        # Linear velocity for each agent:         
        u_i = self.PoseRate[0,Agent_ID]
        Omega_i = self.PoseRate[1,Agent_ID]

        
        # Actual Pose of agent in spaces
        x_act = PoseAct[0,Agent_ID]
        y_act = PoseAct[1,Agent_ID]  
        Theta_act = PoseAct[2,Agent_ID]
        
        # Pose error 
        q_error = np.zeros((3,1))

        # Break down the predicted pose rates in the x,y, theta coordinates 
        # These equations are a breakdown of eq.8a) 

        x_dot_pred = u_i*np.cos(self.PoseMeasPred[2, Agent_ID]) + self.SimParam.gamma[0,0]*self.w_bar[0,Agent_ID]+ np.sum(self.Ki[Agent_ID][0][:]*(PoseMeas[0, Agent_ID]-self.PoseMeasPred[0, Agent_ID]))
        y_dot_pred = u_i*np.sin(self.PoseMeasPred[2, Agent_ID]) + self.SimParam.gamma[1,1]*self.w_bar[1,Agent_ID]+ np.sum(self.Ki[Agent_ID][1][:]*(PoseMeas[1, Agent_ID]-self.PoseMeasPred[1, Agent_ID]))
        theta_dot_pred = Omega_i + np.sum(self.Ki[Agent_ID][2][:]*(PoseMeas[2, Agent_ID]-self.PoseMeasPred[2, Agent_ID]))
        

        x_pred_next = self.PosePred[0, Agent_ID] + x_dot_pred*dt      
        y_pred_next = self.PosePred[1, Agent_ID] + y_dot_pred*dt
        Theta_pred_next = self.PosePred[2, Agent_ID] + theta_dot_pred*dt       
        # This will wrap the Theta_pred value between -pi and pi
        Theta_pred_next = (Theta_pred_next + np.pi)%(2*np.pi)-np.pi

        # Update the predicted states for agent i 
        self.PosePred[0, Agent_ID] = x_pred_next
        self.PosePred[1, Agent_ID] = y_pred_next
        self.PosePred[2, Agent_ID] = Theta_pred_next 
        
        # eq 8c) 
        q_error[0] = x_act-self.PosePred[0, Agent_ID]
        q_error[1] = y_act-self.PosePred[1, Agent_ID]
        q_error[2] = Theta_act-self.PosePred[2, Agent_ID]

        q_error_transpose = np.transpose(q_error)
        self.Pi[Agent_ID-1][:][:] = np.cov(q_error*q_error_transpose)

        # eq 8b)
        self.Ai = np.array([[0, 0, -u_i*np.sin(self.PosePred[2, Agent_ID])],
                            [0,0, -u_i*np.cos(self.PosePred[2, Agent_ID])],
                            [0, 0, 0]])
        
        
        # Calculating the covariance matrix rate of change:
        Ai_Transpose = np.transpose(self.Ai)
        

        self.Pi_dot[Agent_ID-1][:][:] = self.Ai*self.Pi[Agent_ID -1][:][:]+self.Pi[Agent_ID -1][:][:]*Ai_Transpose-self.Ki[Agent_ID-1][:][:]*self.SimParam.H_i*self.Pi[Agent_ID-1][:][:]+ np.dot(np.dot(self.SimParam.gamma,self.SimParam.Pw_i),np.transpose(self.SimParam.gamma) )
        # Predicted Measurement 
        x_meas_pred =np.asscalar(self.PosePred[0, Agent_ID]  + self.SimParam.v_i[0, Agent_ID])
        y_meas_pred = np.asscalar(self.PosePred[1, Agent_ID]  + self.SimParam.v_i[1, Agent_ID])
        Theta_meas_pred = np.asscalar(self.PosePred[2, Agent_ID]  + self.SimParam.v_i[2, Agent_ID])
        Theta_meas_pred = (Theta_meas_pred + np.pi)%(2*np.pi)-np.pi


        self.PoseMeasPred[0,Agent_ID] = x_meas_pred
        self.PoseMeasPred[1,Agent_ID] = y_meas_pred
        self.PoseMeasPred[2,Agent_ID] = Theta_meas_pred

        
        self.Pi[Agent_ID-1][:][:] = self.Pi[Agent_ID-1][:][:]+self.Pi_dot[Agent_ID-1][:][:]*dt 
        
        # Intermediate step used to check if calculation is a division by zero
        B = -self.Pi[Agent_ID-1][:][:]*np.transpose(self.SimParam.H_i)
        C = B*pow(self.SimParam.H_i,-1)
        C[np.isnan(C)]=0
        
        self.Ki[Agent_ID-1][:][:] = C

    def FieldGen(self, xAgent, yAgent,pose_neigh, Agent_ID):
        # ======== Reset the ref field generation values for each agent ========
        # variables used in the potential field calculation 
        self.Fi_x = 0
        self.Fi_y = 0
        self.Fi   = np.zeros((2,1))

        # Partial derivatives for attractive field
        self.dFi_Ax_dx = 0
        self.dFi_Ax_dy = 0
        self.dFi_Ay_dx = 0
        self.dFi_Ay_dy = 0

        # Partial derivatives for repulsive field  
        self.dFi_R_ijx_dx = np.zeros((1,self.NumAgents))
        self.dFi_R_ijx_dy = np.zeros((1,self.NumAgents))
        self.dFi_R_ijy_dx = np.zeros((1,self.NumAgents))
        self.dFi_R_ijy_dy = np.zeros((1,self.NumAgents))
  
        Fi_R_ijx  = np.zeros((1,self.NumAgents))
        Fi_R_ijy  = np.zeros((1,self.NumAgents))
            
        
        # Function used for reference field calculations    
        
        #==================== Attractive Field  ==========================

        # Goal location
        xGoal  = self.SimParam.GoalPose[Agent_ID,0]
        yGoal  = self.SimParam.GoalPose[Agent_ID,1]
        
        # Set of equations related to the attractive field
        # Nomenclature:
        # Fi_A: Attractive force as defined by Eqs 6a and 6b 
        GoalDist = LA.norm(np.array([xAgent-xGoal,yAgent-yGoal])) 

        Fi_Ax = np.asscalar(-(xAgent-xGoal)/(pow(GoalDist,2)))
        Fi_Ay = np.asscalar(-(yAgent-yGoal)/(pow(GoalDist,2)))
        Fi_A = np.array([[Fi_Ax], [Fi_Ay]])


        # Partial derivatives of the attractive field: 
        # Dummy variable 
        A = pow(pow((xAgent-xGoal),2)+pow((yAgent-yGoal),2),2)
        
        # x-direction 
        self.dFi_Ax_dx = (pow((xAgent-xGoal),2)-pow((yAgent-yGoal),2))/A 
        self.dFi_Ax_dy = 2*(xAgent-xGoal*(yAgent-yGoal))/A

        # y-direction 
        self.dFi_Ay_dx = 2*(xAgent-xGoal*(yAgent-yGoal))/A
        self.dFi_Ay_dy = (pow((xAgent-xGoal),2)-pow((yAgent-yGoal),2))/A

 
        #==================== Repulsive Field  ==========================
        
        #  Fi_R_ij: Repulsive force of agent j w.r.t agent i as defined by ref [27]
        #  eqn 20a and 20b.

        # Constant: 
        # rj = [xj; yj] Location of agent we want to move away from.should be a matrix of agents up to k agents

        # Variable: 
        # r= [x;y] Current location for agent i. 

        # set up for loop for both the repulsive for and its partial deriv 
        xNeigh = np.zeros((1,self.NumAgents))
        yNeigh = np.zeros((1,self.NumAgents))

        Fi_R_ij = np.zeros([2, self.NumAgents])

        xNeigh = np.array([pose_neigh[0,:]])
        yNeigh = np.array([pose_neigh[1,:]])


        for a in range (0,self.NumAgents):
            
            if a != Agent_ID  and np.any(xNeigh[0,a]) == True and np.any(yNeigh[0,a]) == True: 
                
                # Operation common to both the repulsive field and the partial derivatives:
                # Use A as a dummy variable
                A = LA.norm(np.array( [xAgent-xNeigh[0,a], yAgent-yNeigh[0,a]]))
                Fi_R_ijx[0,a] = (xAgent-xNeigh[0,a])/A
                Fi_R_ijy[0,a] = (yAgent-yNeigh[0,a])/A                  
          
                # Partial derivatives of the repulsive field:
                # x-direction
                self.dFi_R_ijx_dx[0,a] = pow(A,-1)*(1-pow(xAgent-xNeigh[0,a],2)*pow(A,-2))
                self.dFi_R_ijx_dy[0,a] = -1/(2*pow(A,(3)))
                # y-direction
                self.dFi_R_ijy_dx[0,a] = -1/(2*pow(A,(3)))
                self.dFi_R_ijy_dy[0,a] = pow(A,-(1))*(1-pow(yAgent-yNeigh[0,a],2)*pow(A,-2))
        Fi_R_ij = np.array([Fi_R_ijx,Fi_R_ijy]) 
              
           
        #==================== Reference field calculation  ==========================
        # Reference field force

        # Nomenclature:
        #  Fi_x: Resultant force in the x-direction as a combination of attractive,
        #  repulsive forces and bump function.
        #  Fi_y: Resultant force in the y-direction as a combination of attractive,
        #  repulsive forces and bump function.
        # Fi should be a 2x1 with Fi(1,1) being x direction and Fi(2,1) beingy direction. 

        # Dummy Variable for attractive field calculation
        A = np.prod(np.ones((1,self.NumAgents))-self.sigma_ij)*Fi_A
        # Dummy variable for repulsive field calculation
        B = np.sum(np.reshape(self.sigma_ij*Fi_R_ij, (2, self.NumAgents)), axis =1)
        
        self.Fi[0,0]= A[0,0] +B[0]
        self.Fi[1,0]= A[1,0] +B[1]

    def RefField(self, PoseAct, pose_neigh, ui_n, Agent_ID,Neigh_ID):        
        # ========Reset the reference field values for each agent.========
        # x-y components for normalized reference field function Fi
        self.Fi_pnx = 0
        self.Fi_pny = 0 

        # Orientation of perturbed reference field for each agent 
        self.psi_p = 0

        # Partial derivatives of the normalized perturbed reference field Fi_n
        self.dFi_pnx_dx = 0
        self.dFi_pnx_dy = 0
        self.dFi_pny_dx = 0
        self.dFi_pny_dy = 0
        
        xAgent = PoseAct[0,Agent_ID]
        yAgent = PoseAct[1,Agent_ID]
       
        # Predicted agent location 
        xAgent_pred = self.PosePred[0,Agent_ID]
        yAgent_pred = self.PosePred[1,Agent_ID]

        # Reorganize w_bar 
        w_bar = np.array([[self.w_bar[0,Agent_ID]],[self.w_bar[1,Agent_ID]]])
       
        # Calling sigma generation function  
        dr = self.SimParam.dr 
        dc = self.SimParam.dc
        dm = self.SimParam.dm

        dik = 0


        self.sigma_ij     = np.zeros((1,self.NumAgents))

        # setting up the sigma generator based on the nearest neighbour to the current agent
        if Neigh_ID  != []:
            for index in range(0,len(Neigh_ID)):    
                S = Neigh_ID[index]
                dik = LA.norm([pose_neigh[0,S-1]-xAgent,pose_neigh[1,S-1]-yAgent])
                a = -2/pow((dr-dc),3)
                b = 3*(dr+dc)/pow((dr-dc),3)
                c = -(6*dr*dc)/(pow((dr-dc),3))
                d = (pow(dc,2))*(3*dr-dc)/( pow((dr-dc),3))

                if dik >= dm and dik <= dr: 
                    self.sigma_ij[0,S] =1

                elif dik >= dr and dik <= dc: 
                    self.sigma_ij[0,S] = a*pow(dik,3)+b*pow(dik,2)+c*dik+d
                
                elif dik > dc:
                    self.sigma_ij[0,S] = 0


        # This will define the variable parameters for ui_c which depends on the actual position of the agents in space       
        self.FieldGen(xAgent, yAgent, pose_neigh,Agent_ID)
              
        Fi_qx = self.Fi[0,0]
        Fi_qy = self.Fi[1,0]
        Fi_q = self.Fi

        Fi_p = ui_n[0,Agent_ID]*Fi_q/LA.norm(Fi_q)-w_bar
        self.ui_c = LA.norm(Fi_p)


        # The second time around we will use the predicted pose since the values used to calculate the partial derivaties of our system depened on the predicted pose of agent i
        self.FieldGen(xAgent_pred, yAgent_pred, pose_neigh,Agent_ID)
       
        Fi_q_pred = self.Fi


        # # #This is differrent than the previous perturbed reference field, this uses the predicted pose which is why it has the label _Pred        
        Fi_p_Pred = ui_n[0,Agent_ID]*Fi_q_pred/LA.norm(Fi_q_pred)-w_bar 
        Fi_pn_pred = Fi_p_Pred/LA.norm(Fi_p_Pred)


        self.Fi_pnx = Fi_pn_pred[0,0]
        self.Fi_pny = Fi_pn_pred[1,0]

        # Use the perturbed reference field to calculate psi_p.
        self.psi_p = np.arctan2(self.Fi_pny,self.Fi_pnx)       

        # The first set of partial derivatives are calculated based on having the
        # predicted value for (x,y) from the observer model 
        # Partial derivatives:
        
        # x direction 
        dFi_x_dx = (np.prod(np.ones(self.NumAgents)-self.sigma_ij))*self.dFi_Ax_dx + np.sum(self.sigma_ij*self.dFi_R_ijx_dx, axis =1)
        dFi_x_dy = (np.prod(np.ones(self.NumAgents)-self.sigma_ij))*self.dFi_Ax_dy + np.sum(self.sigma_ij*self.dFi_R_ijx_dy, axis =1)
        
        # y direction
        dFi_y_dx = (np.prod(np.ones(self.NumAgents)-self.sigma_ij))*self.dFi_Ay_dx + np.sum(self.sigma_ij*self.dFi_R_ijy_dx, axis =1)
        dFi_y_dy = (np.prod(np.ones(self.NumAgents)-self.sigma_ij))*self.dFi_Ay_dy + np.sum(self.sigma_ij*self.dFi_R_ijy_dy, axis =1)
        

        
        # Partial derivatives of the perturbed reference field Fi: 
        # dummy variable constant to all equations
        A = pow(LA.norm(Fi_q_pred),2)
        
        # x-direction
        dFixp_dx = ui_n[0,Agent_ID]*(dFi_x_dx*pow(A,1/2)-Fi_q_pred[0,0]*(1/2)*pow(A,-1/2)*(2*Fi_q_pred[0,0]*dFi_x_dx+2*Fi_q_pred[1,0]*dFi_y_dx))/A
        dFixp_dy = ui_n[0,Agent_ID]*(dFi_x_dy*pow(A,1/2)-Fi_q_pred[0,0]*(1/2)*pow(A,-1/2)*(2*Fi_q_pred[0,0]*dFi_x_dy+2*Fi_q_pred[1,0]*dFi_y_dy))/A
        
        # y-direction
        dFiyp_dx = ui_n[0,Agent_ID]*(dFi_y_dx*pow(A,1/2)-Fi_q_pred[1,0]*(1/2)*pow(A,-1/2)*(2*Fi_q_pred[0,0]*dFi_x_dx+2*Fi_q_pred[1,0]*dFi_y_dx))/A
        dFiyp_dy = ui_n[0,Agent_ID]*(dFi_y_dy*pow(A,1/2)-Fi_q_pred[1,0]*(1/2)*pow(A,-1/2)*(2*Fi_q_pred[0,0]*dFi_x_dy+2*Fi_q_pred[1,0]*dFi_y_dy))/A

        # Partial derivatives of the normalized/perturbed reference field Fi: 
        # dummy variable constant to all equations
        A = pow(LA.norm(Fi_p_Pred),2)
        
        # x-direction
        self.dFi_pnx_dx = (dFixp_dx*pow(A,1/2)-Fi_p_Pred[0,0]*(1/2)*pow(A,-1/2)*(2*Fi_p_Pred[0,0]*dFixp_dx+2*Fi_p_Pred[1,0]*dFiyp_dx))/A
        self.dFi_pnx_dy = (dFixp_dy*pow(A,1/2)-Fi_p_Pred[0,0]*(1/2)*pow(A,-1/2)*(2*Fi_p_Pred[0,0]*dFixp_dy+2*Fi_p_Pred[1,0]*dFiyp_dy))/A
        
        # y-direction
        self.dFi_pny_dx = (dFiyp_dx*pow(A,1/2)-Fi_p_Pred[0,0]*(1/2)*pow(A,-1/2)*(2*Fi_p_Pred[0,0]*dFixp_dx+2*Fi_p_Pred[1,0]*dFiyp_dx))/A
        self.dFi_pny_dy = (dFiyp_dy*pow(A,1/2)-Fi_p_Pred[0,0]*(1/2)*pow(A,-1/2)*(2*Fi_p_Pred[0,0]*dFixp_dy+2*Fi_p_Pred[1,0]*dFiyp_dy))/A

if __name__ == '__main__':
    rospy.init_node('trajectory_planner',disable_signals = True)
    AgentModelNode()
    rospy.spin()  