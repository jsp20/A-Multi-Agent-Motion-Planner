#!/usr/bin/env python

import rospy 
from gazebo_msgs.srv import *
from Simulation_Parameters import SwarmSimParams

# ============================================================================================================
# Agents can be randomly spawned using a gazebo setup script. 
# The purpose of this script is to quickly change the starting pose of the agents
# without having to restart the ROS simulation and having to edit the TurtleBot2 setup script.
# ============================================================================================================

class PoseSetUp(object):

  def __init__(self):
    rospy.init_node('robot_env')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state_service = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

    SimParams = SwarmSimParams()
    StartPoses = SimParams.StartPose

    # Set pose for robot1
    PoseRobot1 =  SetModelStateRequest()

    PoseRobot1.model_state.model_name = "1_robot"
    PoseRobot1.model_state.pose.position.x = StartPoses[0,0]
    PoseRobot1.model_state.pose.position.y = StartPoses[0,1]
    PoseRobot1.model_state.pose.position.z = 0
    PoseRobot1.model_state.pose.orientation.w = 0
    PoseRobot1.model_state.pose.orientation.x = 0
    PoseRobot1.model_state.pose.orientation.y = 0
    PoseRobot1.model_state.pose.orientation.z = 0
    PoseRobot1.model_state.twist.linear.x = 0
    PoseRobot1.model_state.twist.linear.y = 0
    PoseRobot1.model_state.twist.linear.z = 0
    PoseRobot1.model_state.twist.angular.x = 0
    PoseRobot1.model_state.twist.angular.y = 0
    PoseRobot1.model_state.twist.angular.z = 0
    PoseRobot1.model_state.reference_frame = "world"

    result = set_state_service(PoseRobot1)
    
    # Set pose for robot
    PoseRobot2 =  SetModelStateRequest()
    PoseRobot2.model_state.model_name = "2_robot"
    PoseRobot2.model_state.pose.position.x = StartPoses[1,0]
    PoseRobot2.model_state.pose.position.y = StartPoses[1,1]
    PoseRobot2.model_state.pose.position.z = 0
    PoseRobot2.model_state.pose.orientation.w = 0
    PoseRobot2.model_state.pose.orientation.x = 0
    PoseRobot2.model_state.pose.orientation.y = 0
    PoseRobot2.model_state.pose.orientation.z = 0
    PoseRobot2.model_state.twist.linear.x = 0
    PoseRobot2.model_state.twist.linear.y = 0
    PoseRobot2.model_state.twist.linear.z = 0
    PoseRobot2.model_state.twist.angular.x = 0
    PoseRobot2.model_state.twist.angular.y = 0
    PoseRobot2.model_state.twist.angular.z = 0
    PoseRobot2.model_state.reference_frame = "world"

    result = set_state_service(PoseRobot2)

    # Set pose for robot3
    PoseRobot3 =  SetModelStateRequest()
    PoseRobot3 =  SetModelStateRequest()
    PoseRobot3.model_state.model_name = "3_robot"
    PoseRobot3.model_state.pose.position.x = StartPoses[2,0]
    PoseRobot3.model_state.pose.position.y = StartPoses[2,1]
    PoseRobot3.model_state.pose.position.z = 0
    PoseRobot3.model_state.pose.orientation.w = 0
    PoseRobot3.model_state.pose.orientation.x = 0
    PoseRobot3.model_state.pose.orientation.y = 0
    PoseRobot3.model_state.pose.orientation.z = 0
    PoseRobot3.model_state.twist.linear.x = 0
    PoseRobot3.model_state.twist.linear.y = 0
    PoseRobot3.model_state.twist.linear.z = 0
    PoseRobot3.model_state.twist.angular.x = 0
    PoseRobot3.model_state.twist.angular.y = 0
    PoseRobot3.model_state.twist.angular.z = 0
    PoseRobot3.model_state.reference_frame = "world"

    result = set_state_service(PoseRobot3)

    # Set pose for robot4
    PoseRobot4 =  SetModelStateRequest()
    PoseRobot4.model_state.model_name = "4_robot"
    PoseRobot4.model_state.pose.position.x = StartPoses[3,0]
    PoseRobot4.model_state.pose.position.y = StartPoses[3,1]
    PoseRobot4.model_state.pose.position.z = 0
    PoseRobot4.model_state.pose.orientation.w = 0
    PoseRobot4.model_state.pose.orientation.x = 0
    PoseRobot4.model_state.pose.orientation.y = 0
    PoseRobot4.model_state.pose.orientation.z = 0
    PoseRobot4.model_state.twist.linear.x = 0
    PoseRobot4.model_state.twist.linear.y = 0
    PoseRobot4.model_state.twist.linear.z = 0
    PoseRobot4.model_state.twist.angular.x = 0
    PoseRobot4.model_state.twist.angular.y = 0
    PoseRobot4.model_state.twist.angular.z = 0
    PoseRobot4.model_state.reference_frame = "world"

    result = set_state_service(PoseRobot4)

if __name__ == '__main__':
  PoseSetUp()