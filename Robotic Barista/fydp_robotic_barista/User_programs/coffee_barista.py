#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.


############################# import files ################################
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib; roslib.load_manifest('robotiq_c_model_control')
import math
##import time
import os
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from time import sleep
from std_msgs.msg import String
from robotiq_force_torque_sensor.msg import ft_sensor
from robotiq_force_torque_sensor.srv import sensor_accessor
sys.path.insert(0, '/home/robot2/catkin_ws/src/robotiq/robotiq_force_torque_sensor/nodes/')
import PushInClose_PullUpOpen
import basic_functions


#########################  main program ########################
def main_program(group,pub,command):
  basic_functions.presetToSafePosition(group,pub)




############################ setup ####################################


def run():
  pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)
  command = outputMsg.CModel_robot_output();

  print "============ setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('CoffeeMakingByXYZ',
                  anonymous=True)

  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  group = moveit_commander.MoveGroupCommander("manipulator")

  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

 
  ## print Basic Information
  
  print "============ Reference frame: %s" % group.get_planning_frame()

  print "============ End effector: %s" % group.get_end_effector_link()

  print "============ Robot Groups:"
  print robot.get_group_names()

  print "Printing robot state:"
  print robot.get_current_state()
  
  print "ready?"
  
  raw_input()  ##Safety@Sampson  

  

  
  group.set_max_acceleration_scaling_factor(0.5)
 
  main_program(group,pub,command)
  	##better use a while loop to call test when user input
  	
  print "finished"
  
  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  collision_object = moveit_msgs.msg.CollisionObject()

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()


  print "============ STOPPING"


if __name__=='__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass

