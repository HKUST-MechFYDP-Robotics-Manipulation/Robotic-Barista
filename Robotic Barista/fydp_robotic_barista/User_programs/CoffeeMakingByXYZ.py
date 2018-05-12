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


################ variables #######################


################# relative positions (corresponding to object size) [X,Y,Z] ####################3
cup= [0.045,0,0]
pitcher =[0.0,0.160,0.045]
pouring_angle= [45.0,45.0] ##[starting angle, angle change during pouring]

############################# position of target objects ##############################


# [X,Y,Z, gripperOpenPosition,gripperClosePosition] 


corridorX = -0.330 

milkButton=[-0.726526133922,-0.206339196982+0.003,0.205528717435,255,255] ##orientation:   x: 0.36312356609  y: 0.886509715919  z: -0.115298645835  w: 0.262579552896; safe position before pressing is y-0.02


cupInitial = [-0.45624354087,-0.153136040213-0.022, 0.163172543461,170,255] ##wrist1=-3.75 /   RotateAboutAxisBy(group,'y',35,-1)

cupAtCoffeeMachine=[-0.467265857071,0.185898942171+0.05
,cupInitial[2]+0.073,170,255]

button = [-0.442158729479-0.03,0.247907524794+0.05-0.015+0.002,0.494345745586-0.006-0.005,255,255]

centerOfCoffeeInProcess = [corridorX-0.05-cup[0],0.554912640704,0.163172543461] 

cupInProcess=[corridorX-0.05, 0.575358874499,0.158609581509+0.01-0.002,170,255] 

milkStir=[-0.873050993552,0.0284711822576,0.445954426589,190,255]

milkInitial = [-0.779166435869,-0.230728775588, 0.15411873922-0.005,160,220] #[-0.779166435869,-0.22976454756, 0.144032574438,170,197]

startPouring=[-0.411341458505-0.005,0.747606041307-0.01,0.134455418058+0.01,255,255]  ##need to calibrate first !!!!!!!!!!!!  x=-0.374055911422/-0.42582550801/-0.43815478438,y=0.733775661727,z=0.149425223964

milkFinal = [-0.370240463326-0.02,0.832926110489, 0.0490348509667,150,220]

cupFinal = [-0.45624354087,-0.153136040213-0.022, 0.158609581509+0.01-0.002,170,255]  ## may change later

corridorZ= cupAtCoffeeMachine[2]+0.01

temporary = []

CupPositionFile = "test.txt"



########################### GUI (android) ###################################

## publish to topic '/talker_ready' from ROS to Android
## publish msg "ready"
def talker_ready():
    pub = rospy.Publisher('/talker_ready', String, queue_size=10)
    rate = rospy.Rate(1) # rate unit:hz
    ##while not rospy.is_shutdown():
      ##hello_str = "hello world %s" % rospy.get_time()
    hello_str = "ready"
    for i in range(0,5) :
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      rate.sleep()	

 ## subcribe to topic 'chatter'
  ## execute when a=0
  

def receiveOrder(latteArtPattern):
  print "waiting"
  print latteArtPattern
  while latteArtPattern=="0" :
    msg = rospy.wait_for_message('chatter',String)
    print msg.data
    latteArtPattern= msg.data
  return latteArtPattern

######################### read position from files created by AprilTag ##########################
def inputCupPositionFromFile(fileName):
  f = open(fileName, "r")
  x = float (f.readline())
  y = float (f.readline())
  z = float (f.readline())
  print x
  print y
  print z

  cupInitial = [x+0.185868379,y-0.032752103,z+0.15868447,170,255]
  
  print cupInitial

  return cupInitial


############################# gripper control ############################

def gactive(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150						##force need to be adjusted later
  pub.publish(command)
  rospy.sleep(2)
  print "gripper activated"
  return command

def greset(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 0
  pub.publish(command)
  print "reset"
    
def gposition(pub,command,position):   ##0=open, 255=close
  command.rPR = position
  pub.publish(command)   
  print position
  return command
  
#################### basic XYZ and joint motions of UR10  ########################

def moveJointTo(group,jointNumber,value):
  group.set_max_velocity_scaling_factor(0.1*2)
  group_variable_values = group.get_current_joint_values()
  group_variable_values[jointNumber] =value
  group.go(group_variable_values)
  return group.get_current_joint_values()

def moveTo(group,axis,value) : ## direction = 'x'/'y'/'z' for fast speed and 'sx'/'sy'/'sz' for slow speed
  group.set_max_velocity_scaling_factor(0.1*2)
  pose_target = group.get_current_pose()	
  if axis is'x'or axis is'sx' or axis is'mx' :
    pose_target.pose.position.x=value
  if axis is'y'or axis is'sy' or axis is'my' :
    pose_target.pose.position.y=value
  if axis is'z'or axis is'sz' or axis is'mz' :
    pose_target.pose.position.z=value
  if axis is'sx'or axis is'sy'or axis is'sz' :
    group.set_max_velocity_scaling_factor(0.005*2)
  if axis is'mx'or axis is'my'or axis is'mz' :
    group.set_max_velocity_scaling_factor(0.05)
  group.go(pose_target)
  return group.get_current_pose().pose.position

def straightMoveToXY(group,x,y):
  group.set_max_velocity_scaling_factor(0.01)
  waypoints=[]
  wpose = group.get_current_pose().pose
  if(x- group.get_current_pose().pose.position.x)==0:
    dx=0
    dy=(y-group.get_current_pose().pose.position.y)/100
  else:
    slope= (y-group.get_current_pose().pose.position.y)/(x- group.get_current_pose().pose.position.x)
    dx=(x- group.get_current_pose().pose.position.x)/100
    dy=dx*slope
  while abs(x- wpose.position.x)>abs(dx) :
    wpose.position.x+=dx
    wpose.position.y+=dy
    waypoints.append(copy.deepcopy(wpose))
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  #raw_input()  ##Safety@Sampson  
  group.execute(plan3)
  print group.get_current_pose().pose

def upMoveXYdown(group,x,y):
  moveTo(group,'sz',group.get_current_pose().pose.position.z+0.03)
  moveTo(group,'sx',x)
  moveTo(group,'sy',y)
  moveTo(group,'sz',group.get_current_pose().pose.position.z-0.03)

#################### calculations for circular motions of UR10  ########################

def takeSign(value) : ##return +1 for positive value and -1 for negative value
  if value<0 :
    return -1
  else :
    return 1


def mathRotateAboutAxisBy(group,axis,pose1,pose2,AngleDegree,direction) : #direction=1/-1, enter the same original pose for both pose1 and pose2  ##axis='x'/'y'/'z' 
  original_pose = pose1 
  pose_target = pose2
  w_rotation = direction*math.cos(AngleDegree/360.0*3.14)
  if axis is 'x' :
    x_rotation = (1-w_rotation**2)**0.5
    pose_target.orientation.w = original_pose.orientation.w*w_rotation - original_pose.orientation.x* x_rotation
    pose_target.orientation.x = original_pose.orientation.x*w_rotation + original_pose.orientation.w* x_rotation
    pose_target.orientation.y = original_pose.orientation.y*w_rotation - original_pose.orientation.z* x_rotation
    pose_target.orientation.z = original_pose.orientation.z*w_rotation + original_pose.orientation.y* x_rotation
  if axis is 'y' :
    y_rotation = (1-w_rotation**2)**0.5
    pose_target.orientation.w = original_pose.orientation.w*w_rotation - original_pose.orientation.y* y_rotation
    pose_target.orientation.x = original_pose.orientation.x*w_rotation + original_pose.orientation.z* y_rotation
    pose_target.orientation.y = original_pose.orientation.y*w_rotation + original_pose.orientation.w* y_rotation
    pose_target.orientation.z = original_pose.orientation.z*w_rotation - original_pose.orientation.x* y_rotation
  if axis is 'z' :
    z_rotation = (1-w_rotation**2)**0.5
    pose_target.orientation.w = original_pose.orientation.w*w_rotation - original_pose.orientation.z* z_rotation
    pose_target.orientation.x = original_pose.orientation.x*w_rotation - original_pose.orientation.y* z_rotation
    pose_target.orientation.y = original_pose.orientation.y*w_rotation + original_pose.orientation.x* z_rotation
    pose_target.orientation.z = original_pose.orientation.z*w_rotation + original_pose.orientation.w* z_rotation
  return pose_target

def mathpour(group,tip_x,tip_y,tip_z,t2):             ### being tested
  group.set_max_velocity_scaling_factor(0.0005)
    
  t1= math.atan(4/15.5)
  r = 0.155 # from tip to handle pitch[2]
  theta=0
  dtheta = 1.0/180.0*3.14/5

  waypoints=[]
  waypoints.append(group.get_current_pose().pose)
  wpose = group.get_current_pose().pose
  
  while theta<3.14*2*3 :
    wpose.position.x= size/2*math.sin(theta) + centerX 
    wpose.position.y= size/(3*3.14)* theta - size + centerY
    wpose.position.z = r*math.sin(t1+t2/180.0*3.14) + tip_z
    waypoints.append(copy.deepcopy(wpose))
    theta = theta + dtheta
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  ##raw_input()  ##Safety@Sampson  
  
  group.execute(plan3)
  
  print "finish draw"
  tip_y = tip_y + size/(3*3.14)* theta - size + centerY
  tip_x = tip_x + size/2*math.sin(theta) + centerX

  final_grip_pose_z = r*math.sin(t1+t2/180.0*3.14) + tip_z
  final_grip_pose_y = r*math.cos(t1+t2/180.0*3.14) + tip_y
  


 


 
def mathArcAboutAxis(group,waypoints,axis,CenterOfCircleI1,CenterOfCircleI2,AngleDegree,direction,turn_or_not) : ##must be less than 90 degrees, enter "CenterOfCircleI1,CenterOfCircleI2" by the order of "x,y" or "y,z" or "z,x"  ##axis='x'/'y'/'z'  ##direction=1/-1  ##enter 'yes' for turn_or_not if the angle has to change accordingly
  waypoints.append(group.get_current_pose().pose)
  originalPose =group.get_current_pose().pose
  wpose = group.get_current_pose().pose
  #positionI1 = null
  #positionI2 = null
  if axis is 'x' :
    positionI1 = wpose.position.y
    positionI2 = wpose.position.z
  if axis is 'y' :
    positionI1 = wpose.position.z
    positionI2 = wpose.position.x
  if axis is 'z' :
    positionI1 = wpose.position.x
    positionI2 = wpose.position.y

  radiusOfCircle=((positionI1-CenterOfCircleI1)**2+(positionI2-CenterOfCircleI2)**2)**0.5  #Pyth. Theorm
  distance = radiusOfCircle/50
  currentAngle= 0
  i=0
  while currentAngle<AngleDegree/180.0*3.14:   
    if positionI2-CenterOfCircleI2==0 :
      positionI2 -= direction*distance/10  
    else :
      slope = -(positionI1 -CenterOfCircleI1)/(positionI2 -CenterOfCircleI2)
      deltaI1 = distance/(1+ slope**2)**0.5
      deltaI2 = distance/(1+ 1/slope**2)**0.5      
      positionI1 -= direction*takeSign(positionI2 -CenterOfCircleI2)*deltaI1
      positionI2 += direction*takeSign(positionI1 -CenterOfCircleI1)*deltaI2
      if axis is 'x' :
        wpose.position.y = positionI1
        wpose.position.z = positionI2 
        original_positionI1 = originalPose.position.y
        original_positionI2 = originalPose.position.z
      if axis is 'y' :
        wpose.position.z = positionI1
        wpose.position.x = positionI2 
        original_positionI1 = originalPose.position.z
        original_positionI2 = originalPose.position.x
      if axis is 'z' :
        wpose.position.x = positionI1
        wpose.position.y = positionI2 
        original_positionI1 = originalPose.position.x
        original_positionI2 = originalPose.position.y
      
      cosCurrentAngle = ((positionI1-CenterOfCircleI1)*(original_positionI1-CenterOfCircleI1)+(positionI2-CenterOfCircleI2)*(original_positionI2-CenterOfCircleI2))/radiusOfCircle**2      ##cosine law
      if cosCurrentAngle>1 :
        cosCurrentAngle =1
      if cosCurrentAngle<-1 :
        cosCurrentAngle = -1
      AngleChange= math.acos(cosCurrentAngle)-currentAngle
      currentAngle= math.acos(cosCurrentAngle)
      print "currentAngle:"
      print currentAngle *180/3.14
      if turn_or_not is 'yes' :
        wpose = mathRotateAboutAxisBy(group,axis,wpose,wpose,AngleChange*180/3.14,direction)
      waypoints.append(copy.deepcopy(wpose))
      i+=1
      print i
  
  return waypoints




#################### useful circular motions of UR10  ########################
 
def RotateAboutAxisBy(group,axis,AngleDegree,direction) : ##axis='x'/'y'/'z'  ##direction=1/-1
  group.set_pose_target(mathRotateAboutAxisBy(group,axis,group.get_current_pose().pose,group.get_current_pose().pose,AngleDegree,direction))
  plan1 = group.plan()
  #rospy.sleep(2)
  print "ready?"
  #raw_input()  ##Safety@Sampson  
  group.execute(plan1)

def DrawArcAboutAxis(group,axis,CenterOfCircleI1,CenterOfCircleI2,AngleDegree,direction) : ##must be less than 90 degrees, enter "CenterOfCircleI1,CenterOfCircleI2" by the order of "x,y" or "y,z" or "z,x"   ##axis='x'/'y'/'z'  ##direction=1/-1
  waypoints = []
  #group.set_max_velocity_scaling_factor(0.05)
  waypoints= mathArcAboutAxis(group,waypoints,axis,CenterOfCircleI1,CenterOfCircleI2,AngleDegree,direction,no)
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  raw_input()  ##Safety@Sampson  
  group.execute(plan3)
  print group.get_current_pose().pose

def TurnArcAboutAxis(group,axis,CenterOfCircleI1,CenterOfCircleI2,AngleDegree,direction) : ##must be less than 90 degrees, enter "CenterOfCircleI1,CenterOfCircleI2" by the order of "x,y" or "y,z" or "z,x"  ##axis='x'/'y'/'z'  ##direction=1/-1
  waypoints = []
  #group.set_max_velocity_scaling_factor(0.05)
  waypoints= mathArcAboutAxis(group,waypoints,axis,CenterOfCircleI1,CenterOfCircleI2,AngleDegree,direction,'yes')
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  #raw_input()  ##Safety@Sampson  
  group.execute(plan3)
  print group.get_current_pose().pose
##################### calculations for Latte art patterns (centerX should usually be the center of cup,size should be a bit smaller than the radius of the cup) ############################


def mathRsinKTheta(group,waypoints,centerX,centerY,size,k,fractionOfCompletion,direction): ## for drawing flower and circles or ellipse
  waypoints.append(group.get_current_pose().pose)
  originalPose =group.get_current_pose().pose
  wpose = group.get_current_pose().pose
  r=0.0
  theta=0.0
  a=0.0
  if k%2==0:
    a=2.0
  else :
    a=1.0
  while theta<3.14*a*fractionOfCompletion :
    dtheta=1.0/180*3.14/40#(size*200*5.0/4.0-10.0)
    theta=theta+dtheta
    r=size*math.sin(k*theta)
    wpose.position.x=direction*r*math.cos(theta)+centerX
    wpose.position.y=r*math.sin(theta)+centerY
    waypoints.append(copy.deepcopy(wpose))
    print "r:"
    print r
    print "theta:"
    print theta
  print "finish mathRsinKTheta"
  return waypoints

def mathflower(group,waypoints,centerX,centerY,size):##not use
  wpose = group.get_current_pose().pose
  r=0.0
  theta=0.0
  k=5.0
  a=0.0
  if k%2==0:
    a=2.0
  else :
    a=1.0
  while theta<3.14*a*2 :
    dtheta=1.0/180*3.14/40#(size*200*5.0/4.0-10.0)
    theta=theta+dtheta
    r=size*math.sin(k*theta)
    wpose.position.x=r*math.cos(theta)+centerX
    wpose.position.y=r*math.sin(theta)+centerY
    waypoints.append(copy.deepcopy(wpose))
    print "r:"
    print r
    print "theta:"
    print theta
  print "finish math"
  return waypoints

def mathleaf(group,waypoints,centerX,centerY,size): ##not yet adjust or used
  waypoints.append(group.get_current_pose().pose)
  originalPose =group.get_current_pose().pose
  wpose = group.get_current_pose().pose
  r=0.0
  theta=0.0
  k=2.0
  a=0.0
  if k%2==0:
    a=2.0
  else :
    a=1.0
  while theta<3.14*a :
    dtheta=1.0/180*3.14/42.0
    theta=theta+dtheta
    r=size*math.sin(k*theta)
    wpose.position.x=r*math.cos(theta)+centerX
    wpose.position.y=r*math.sin(theta)+centerY
    waypoints.append(copy.deepcopy(wpose))
    print "r:"
    print r
    print "theta:"
    print theta
  print "finish math"
  return waypoints

def mathSwirl(group,waypoints,centerX,centerY,numberOfCycles,size,direction):
  waypoints.append(group.get_current_pose().pose)
  wpose = group.get_current_pose().pose
  r=0.0
  theta=0.0
  theta_0=math.atan((group.get_current_pose().pose.position.y-centerY)/(group.get_current_pose().pose.position.x-centerX))
  if group.get_current_pose().pose.position.x<centerX:
    direction=direction*-1
 
  while theta<3.14*2*numberOfCycles :
    dtheta=1.0/180*3.14/10.0
    theta=theta+dtheta
    r=size*(1-theta/(3.14*2*numberOfCycles))
    wpose.position.x=direction*r*math.cos(theta+theta_0)+centerX
    wpose.position.y=direction*r*math.sin(theta+theta_0)+centerY
    waypoints.append(copy.deepcopy(wpose))
    print "r:"
    print r
    print "theta:"
    print theta
  print "finish math"
  return waypoints

def mathCircle(group,waypoints,startPointX,startPointY,size,fraction_of_completion,direction): ##size is diameter ## draw towards y-direction only 
  waypoints = mathRsinKTheta(group,waypoints,startPointX,startPointY,size,1,fraction_of_completion,direction)
  return waypoints


################ real motion to draw latte art patterns(centerX should usually be the center of cup,size should be a bit smaller than the radius of the cup)  #####################3
def drawFlower(group,centerX,centerY,size):  ##already tested
  waypoints=[]
  waypoints = mathflower(group,waypoints,centerX,centerY,size)
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  raw_input()  ##Safety@Sampson  
  group.execute(plan3)
  print "finish draw"

def drawLuckyLeaf(group,centerX,centerY,size):  ##already tested
  waypoints=[]
  #waypoints = mathRsinKTheta(group,waypoints,centerX,centerY,size,2,1,1)
  poses=[[1,0,-1,0],[0,1,0,-1]]
  for i in range(0,4) :
    upMoveXYdown(group,centerX+size*poses[0][i],centerY+size*poses[1][i])
    straightMoveToXY(group,centerX,centerY)
  print "finish draw"

def drawSwirl(group,centerX,centerY,numberOfCycles,size,direction):  ##already tested
  waypoints=[]
  waypoints = mathSwirl(group,waypoints,centerX,centerY,numberOfCycles,size,direction)
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  group.execute(plan3)
  print "finish draw"
  


def drawWheel(group,centerX,centerY,size):  ##already tested
  waypoints=[]
  upMoveXYdown(group,centerX+size,centerY)
  waypoints = mathSwirl(group,waypoints,centerX,centerY,3,size,1)
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  raw_input()  ##Safety@Sampson  
  group.set_max_velocity_scaling_factor(0.0005)
  group.execute(plan3)
  print "finish draw"
  

def drawHeart(group,centerX,centerY,size): 
  group.set_max_velocity_scaling_factor(0.0005)  
  
  print 'y:' 
  print centerY-size
  moveTo(group,'y',centerY-size)  
  
  waypoints=[]
  waypoints.append(group.get_current_pose().pose)
  wpose = group.get_current_pose().pose
  theta=0
  i = 0
  dtheta = 1.0/180.0*3.14/5
  while i<2:
    theta=0
    while theta<3.14 :
      dx = (-1)**(i)*size*math.sin(theta)
      dy = size*math.cos(theta)
      wpose.position.y= centerY-dy
      wpose.position.x= centerX+dx
      waypoints.append(copy.deepcopy(wpose))
      theta = theta + dtheta
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
    ##raw_input()  ##Safety@Sampson  
    group.execute(plan3)
    moveTo(group,'y',centerY)
    
    if i == 0 :
      moveTo(group,'sz',group.get_current_pose().pose.position.z+0.01)
      moveTo(group,'y',centerY-size)
      moveTo(group,'sz',group.get_current_pose().pose.position.z-0.01)
    i+=1
    print 'theta:'     
    print theta
    print 'i:'
    print i
  print "finish draw"

####def drawHeart(group,centerX,centerY,size):  ##being tested
##  group.set_max_velocity_scaling_factor(0.0005)  
##  
##  print 'y:'
##  print centerY-size
##  moveTo(group,'y',centerY-size)  
##  
##  waypoints=[]
##  waypoints.append(group.get_current_pose().pose)
##  wpose = group.get_current_pose().pose
##  theta=0
##  dtheta = 1.0/180.0*3.14/5
##  while theta<3.14*2*3 :
##    wpose.position.y= size*r*math.sin(theta) + centerY
##    wpose.position.x= size*r*math.cos(theta) - size + centerX
##    waypoints.append(copy.deepcopy(wpose))
##    theta = theta + dtheta
##  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
##  ##raw_input()  ##Safety@Sampson  
##  
##  group.execute(plan3)
##  moveTo(group,'x',centerX-size)
##  print "finish draw"


def drawSingleLeaf(group,centerX,centerY,size):  ##being tested
  group.set_max_velocity_scaling_factor(0.0005)  
  
  print 'x:'
  print centerX-size
  moveTo(group,'y',centerY-size)  
  
  waypoints=[]
  waypoints.append(group.get_current_pose().pose)
  wpose = group.get_current_pose().pose
  theta=0
  dtheta = 1.0/180.0*3.14/5
  while theta<3.14*2*3 :
    wpose.position.x= size/4*2.8*math.sin(theta) + centerX
    wpose.position.y= size/(3*3.14)* theta - size + centerY
    waypoints.append(copy.deepcopy(wpose))
    theta = theta + dtheta
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  ##raw_input()  ##Safety@Sampson  
  
  group.execute(plan3)
  moveTo(group,'y',centerY-size)
  print "finish draw"

def drawCircle(group,startPointX,startPointY,size,fraction_of_completion,direction) : ##already tested  ##by Shan
  waypoints=[]
  waypoints = mathCircle(group,waypoints,startPointX,startPointY,size,fraction_of_completion,direction)
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  group.execute(plan3)


def drawHalfLeaf(group,centerX,centerY,size):  ##being tested
  group.set_max_velocity_scaling_factor(0.0005)  
  i=0
  while i<3 :
    moveTo(group,'y',centerY-size)  
    drawCircle(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,size*2.0*(4.0-i)/5.0,0.5,1)  
    moveTo(group,'y',centerY-size)  
    drawCircle(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,size*2.0*(4.0-i)/5.0,0.5,-1.0) 
    i+=1
    print "i"
    print i

  waypoints=[]
  waypoints.append(group.get_current_pose().pose)
  wpose = group.get_current_pose().pose
  theta=0
  dtheta = 1.0/180.0*3.14/5
  while theta<3.14*2*3 :
    wpose.position.x= size/3*math.sin(theta) + centerX
    wpose.position.y= -(size/3)* (theta/(3*3.14)) - size/3 + centerY
    waypoints.append(copy.deepcopy(wpose))
    theta = theta + dtheta
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  group.execute(plan3)
  moveTo(group,'y',centerY+size)  
  print "finish draw"

def freefallpour(group): ##being tested   ## by shan 

  original_pose= group.get_current_pose()
  waypoints=[]
  waypoints.append(group.get_current_pose().pose)
  originalPose= group.get_current_pose().pose
  wpose = group.get_current_pose().pose
  wpose = mathRotateAboutAxisBy(group,'x',wpose,wpose,45,1)
  angle = 45
  a = 180 - angle- math.atan(14.5/4)/3.14*180
  r = (0.145**2+ 0.04**2)**(0.5)
  #wpose.position.y+= - 0.08*math.sin(angle/180.0*3.14)+0.065/2*(math.cos(angle/180.0*3.14)-1)
  Z_difference= 0.04+r*math.cos(a/180*3.14)
  #wpose.position.z+=0.08
  waypoints.append(copy.deepcopy(wpose))
  
  print angle
  direction_x =1

  raw_input()
  while angle < 80:
    wpose = mathRotateAboutAxisBy(group,'x',wpose,wpose,3,1)
    angle += 3
    if angle>60:
      if (angle-60)%3==0 :
        direction_x =direction_x *-1
      wpose.position.x+= direction_x*0.03
      #wpose.position.y+= 0.00/9 - 0.08*math.sin(angle/180.0*3.14)+0.065/2*(math.cos(angle/180.0*3.14)-1)
    #print 0.00/9 - 0.08*math.sin(angle/180.0*3.14)+0.065/2*(math.cos(angle/180.0*3.14)-1)
    a = 180 - angle- math.atan(14.5/4)/3.14*180
    r = (0.145**2+ 0.04**2)**(0.5)
    wpose.position.z= originalPose.position.z+0.04+r*math.cos(a/180*3.14)-Z_difference-0.06*(angle-45+15)/(90-45+15)#-0.115*math.cos(3.14/2-angle/180.0*3.14)
    
    print angle
    
    waypoints.append(copy.deepcopy(wpose))
  
  raw_input()
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  raw_input()  ##Safety@Sampson  
  
  group.execute(plan3)
  print "finish draw"
  
  raw_input()

  target_pose= group.get_current_pose()
  target_pose.pose.orientation=original_pose.pose.orientation
  group.set_max_velocity_scaling_factor(0.05)
  group.go(target_pose)

def drawRose(group,centerX,centerY,size):
  #angle=0
  long_side=size*3/4
  short_side=size/2
  
  i=0
  pose=[[long_side,short_side,0,short_side],[-short_side,long_side,-short_side,0],[-long_side,-short_side,0,-short_side],[short_side,-long_side,short_side,0],]
  
  while i<4:#angle<3.14*2: #
   
    #x1=long_side*math.cos(angle)-short_side*math.sin(angle)+centerX
    #x2=-short_side*math.sin(angle)+centerX
   # y1=long_side*math.sin(angle)+short_side*math.cos(angle)+centerY
   # y2=short_side*math.cos(angle)+centerY
   # upMoveXYdown(group,x1,y1)
   # straightMoveToXY(group,x2,y2)
   # drawSwirl(group,centerX,centerY,1,short_side,1)
  #  angle+=3.14/2
    upMoveXYdown(group,pose[i][0]+centerX,pose[i][1]+centerY)
    straightMoveToXY(group,pose[i][2]+centerX,pose[i][3]+centerY)
    drawSwirl(group,centerX,centerY,1,short_side,1)
    i+=1


#################### practical motions of UR10  ########################



def presetToSafePosition(group,pub):
  print"move to safe x=-0.34"
  moveTo(group,'x',corridorX)
  print"move to safe z=0.6"
  moveTo(group,'z',0.6)
  print"preset joint"
  group_variable_values = group.get_current_joint_values()
  
  if group_variable_values[0]>-3.14/180*90 and group_variable_values[0]<3.14/180*90: 
    if group_variable_values[1]<-3.14/180*90 :
      if group_variable_values[2]<-3.14/180*90 :
        print"safe case 1"
        moveJointTo(group,1,-3.14/180*90)
        moveJointTo(group,2,-3.14/180*90)
      else:
        print"safe case 2"
        moveJointTo(group,1,-3.14/180*120)
        moveJointTo(group,2,-3.14/180*90)
        moveJointTo(group,1,-3.14/180*90)
    else : 
      print"safe case 3"
      moveJointTo(group,2,-3.14/180*90)
      moveJointTo(group,1,-3.14/180*90)
    moveJointTo(group,0,0)
  else :
    if group_variable_values[1]<-3.14/180*90 :
      print"safe case 4"
      moveJointTo(group,2,3.14/180*90)
      moveJointTo(group,1,-3.14/180*120)
    else : 
      if group_variable_values[2]>3.14/180*90 :
        print"safe case 5"
        moveJointTo(group,1,-3.14/180*120)
        moveJointTo(group,2,3.14/180*90)
      else :
        print"safe case 6"
        moveJointTo(group,1,-3.14/180*60)
        moveJointTo(group,2,-3.14/180*90)
    moveJointTo(group,0,0)
    moveJointTo(group,2,-3.14/180*90)
    moveJointTo(group,1,-3.14/180*90)
  #have come to safe position
  new_group_variable_values = group.get_current_joint_values()
  moveJointTo(group,3,new_group_variable_values[1]+new_group_variable_values[2])
  moveJointTo(group,4,-3.14/180*90)
  moveJointTo(group,5,0)
  print group.get_current_pose().pose
  command=gactive(pub)
  return command

def SafeMoveToXYZ(group,X,Y,Z):
  if abs(group.get_current_pose().pose.position.x-corridorX)>0.3:
    moveTo(group,'x',(group.get_current_pose().pose.position.x+corridorX)/2)
  moveTo(group,'x',corridorX)
  moveTo(group,'z',(group.get_current_pose().pose.position.z+corridorZ)/2)
  moveTo(group,'z',corridorZ)
  if abs(group.get_current_pose().pose.position.y-Y)>0.4:
    moveTo(group,'y',(group.get_current_pose().pose.position.y+Y)/3)
  moveTo(group,'y',(group.get_current_pose().pose.position.y+Y)/2)
  moveTo(group,'y',Y)
  moveTo(group,'z',(group.get_current_pose().pose.position.z+Z)/2)
  moveTo(group,'z',Z)
  if abs(group.get_current_pose().pose.position.x-X)>0.3:
    moveTo(group,'x',(group.get_current_pose().pose.position.x+X)/2)
  moveTo(group,'x',X)
 
def PressMilkButton(group,pub,command,Pose) :
  gposition(pub,command,Pose[3])
  back_pose=group.get_current_pose()
  pose_target=group.get_current_pose()
  pose_target.pose.orientation.w= 0.262579552896
  pose_target.pose.orientation.x= 0.36312356609
  pose_target.pose.orientation.y=0.886509715919
  pose_target.pose.orientation.z=-0.115298645835
  group.set_max_velocity_scaling_factor(0.05)
  group.go(pose_target)
  moveTo(group,'y',Pose[1]-0.02)
  moveTo(group,'x',Pose[0])
  moveTo(group,'z',Pose[2])
  moveTo(group,'sy',Pose[1]+0.002)
  moveTo(group,'y',Pose[1]-0.02)
  moveTo(group,'z',0.6)
  pose_target=group.get_current_pose()
  pose_target.pose.orientation=back_pose.pose.orientation
  group.go(pose_target)
 
def PickUpAt(group,pub,command,Pose):##Pose =[X,Y,Z, gripperOpenPosition,gripperClosePosition] 
  gposition(pub,command,Pose[3])
  SafeMoveToXYZ(group,Pose[0]+0.05,Pose[1],Pose[2])
  moveTo(group,'sx',Pose[0]-0.02)
  #raw_input()  ##convenient to correct pose @Sampson  
  gposition(pub,command,Pose[4])
  rospy.sleep(2)
  moveTo(group,'sz',Pose[2]+0.02)

def ft_sensor_calibration(group,pub,command):  ## assume starting position is that the weight is along y-direction of the ft_sensor
  Fx_sum=0
  Fy_sum=0
  Fz_sum=0
  for x in range(0,100) :
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor)
    Fx_sum+=msg.Fx
  Fx0=Fx_sum/100
  moveJointTo(group,5,-3.14/180*90)
  for x in range(0,100) :
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor)
    Fy_sum+=msg.Fy
  Fy0=Fy_sum/100
  for x in range(0,100) :
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor)
    Fz_sum+=msg.Fz
  Fz0=Fz_sum/100
  moveJointTo(group,5,3.14/180*0)
  msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor)
  print msg.Fx-Fx0
  print msg.Fy-Fy0
  print msg.Fz-Fz0 
  
  stop=0
  while stop<1 :
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor)
    print msg.Fx-Fx0
    print msg.Fy-Fy0
    print msg.Fz-Fz0
    print ((msg.Fx-Fx0)**2+(msg.Fy-Fy0)**2)**0.5
    stop=int(input("continue=0,stop=1"))
  
  
  
 

def PickUpMilkAt(group,pub,command,Pose):##Pose =[X,Y,Z, gripperOpenPosition,gripperClosePosition] 
  gposition(pub,command,Pose[3])
  group.set_max_velocity_scaling_factor(0.1)
  RotateAboutAxisBy(group,'z',90,-1)
  SafeMoveToXYZ(group,Pose[0],Pose[1]-0.05,Pose[2])
  moveTo(group,'sy',Pose[1])
  #raw_input()  ##for calibration
  gposition(pub,command,Pose[4])
  rospy.sleep(2)
  moveTo(group,'z',Pose[2]+0.3)
  group.set_max_velocity_scaling_factor(0.1)
  RotateAboutAxisBy(group,'z',90,1)

def PickUpStick(group,pub,command): ##by shan
  
  moveTo(group,'z',0.274819743234)
  
  RotateAboutAxisBy(group,'x',90,1)
  
  gposition(pub,command,140)
  
  moveTo(group,'y',0.4) 
  moveTo(group,'y',0.256691968817) 
  moveTo(group,'x',-0.385847280109)
  #raw_input()
  moveTo(group,'sx',-0.396436425135)
  #raw_input()
  gposition(pub,command,220)  ##grip stick

  moveTo(group,'sz',0.306061802953)
  moveTo(group,'x',-0.239579710676)
  RotateAboutAxisBy(group,'x',90,1)

  moveTo(group,'y',0.569906071378+0.002)
  moveTo(group,'x',-0.389338962758-0.003)
  
  moveTo(group,'z',0.105957696627+0.002)
  raw_input()
  moveTo(group,'z',0.105957696627-0.004)
  raw_input()

def PlaceStickAt(group,pub,command): ##by shan
  moveTo(group,'z',0.341802540554) 
  RotateAboutAxisBy(group,'x',90,-1)
  moveTo(group,'y', 0.256691968817)#0.251121359488)
  #raw_input()
  moveTo(group,'sx',-0.396940839655)
  #raw_input()
  moveTo(group,'sz',0.282321902464)
  gposition(pub,command,170)  ##place stick

  rospy.sleep(2)
  moveTo(group,'x',-0.326066322692)
  RotateAboutAxisBy(group,'x',90,-1)
  

def PlaceAt(group,pub,command,Pose):
  SafeMoveToXYZ(group,Pose[0],Pose[1],Pose[2]+0.01)
  moveTo(group,'sz',Pose[2]-0.006)
  #raw_input()  ##convenient to correct pose @Sampson 
  gposition(pub,command,Pose[3])
  rospy.sleep(2)
  moveTo(group,'sx',Pose[0]+0.07)

def grabMilkStir(group,pub,command):
  Pose=milkStir
  gposition(pub,command,Pose[3])
  SafeMoveToXYZ(group,Pose[0],Pose[1],Pose[2]+0.15)
  RotateAboutAxisBy(group,'y',90,-1)
  moveTo(group,'z',Pose[2])
  gposition(pub,command,Pose[4])
  rospy.sleep(2)
  moveTo(group,'z',Pose[2]+0.15) #pick up 

  group.set_max_acceleration_scaling_factor(1)
  original_pose= group.get_current_pose()
  TurnArcAboutAxis(group,'y',group.get_current_pose().pose.position.z-0.34,group.get_current_pose().pose.position.x,60,1)
  i=0
  while i<10 :
    moveTo(group,'z',Pose[2]+0.0)
    moveTo(group,'z',Pose[2]+0.10)
    i+=1#shake and drop more milk foam

  group.set_max_acceleration_scaling_factor(0.5)
  moveTo(group,'z',Pose[2]+0.2)
  
  RotateAboutAxisBy(group,'y',35,-1)

  #moveTo(group,'x',Pose[0]+0.17)
  #moveTo(group,'z',Pose[2]+0.05) 
  moveTo(group,'z',0.36045825658) #put down
  #raw_input()  ##convenient to correct pose @Sampson 
  gposition(pub,command,Pose[3])
  rospy.sleep(2)
  moveTo(group,'z',Pose[2]+0.15)
  RotateAboutAxisBy(group,'y',65,1)


def shaking(group,size) :  ## not being used
  waypoints=[]
  originalPose =group.get_current_pose().pose
  wpose = group.get_current_pose().pose
  r=0.0
  theta=0.0
  k=1
  while theta<3.14*3 :
    dtheta=3.14/32
    theta=theta+dtheta
    r=size*math.sin(k*theta)
    wpose.position.x=r*math.cos(theta)+group.get_current_pose().pose.position.x
    wpose.position.y=r*math.sin(theta)+group.get_current_pose().pose.position.y
    waypoints.append(copy.deepcopy(wpose))
    print "r:"
    print r
    print "theta:"
    print theta
  (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01,0)
  raw_input()
  group.execute(plan3)

  
def pouring(group) :  ## being tested for advanced application
  print "pouring"
  group.set_max_velocity_scaling_factor(0.05)
  original_pose=group.get_current_pose().pose
  RotateAboutAxisBy(group,'x',pouring_angle[0],1)
  raw_input() 
  start_angle = pouring_angle[0]
  while start_angle < pouring_angle[0]+29 :
    TurnArcAboutAxis(group,'x',group.get_current_pose().pose.position.y-pitcher[1]*math.cos(start_angle/180.0*3.14)-pitcher[2]*math.sin(start_angle/180.0*3.14),group.get_current_pose().pose.position.z+pitcher[2]*math.cos(start_angle /180.0*3.14)-pitcher[1]*math.sin(start_angle/180.0*3.14),5,1)
    start_angle+=3
    if start_angle > pouring_angle[0]+15:
      rospy.sleep(0.3)
  rospy.sleep(3.5)
  option=2#int(input("rotate=0,shake=1,stop=2"))
  while option<2 :
    if option==0:
      TurnArcAboutAxis(group,'x',group.get_current_pose().pose.position.y-pitcher[1]*math.cos(start_angle/180.0*3.14)-pitcher[2]*math.sin(start_angle/180.0*3.14),group.get_current_pose().pose.position.z+pitcher[2]*math.cos(start_angle /180.0*3.14)-pitcher[1]*math.sin(start_angle/180.0*3.14),5,1)
    if option==1:
      while i<3 :
        i+=1
        moveTo(group,'z',group.get_current_pose().pose.position.z+0.1) 
        moveTo(group,'z',group.get_current_pose().pose.position.z-0.1) 
    
    option=int(input("rotate=0,shake=1,stop=2"))
  group.set_max_acceleration_scaling_factor(0.5)
  back_pose=group.get_current_pose()
  back_pose.pose.orientation =original_pose.orientation
  back_pose.pose.position.y=original_pose.position.y     ###comment this line if dangerous things happen
  group.set_max_velocity_scaling_factor(0.5)
  group.go(back_pose)
  
   
def temporaryTrashForPouring(group) : 
  while (false):
    if start_angle == pouring_angle[0]+9:
      rospy.sleep(1)
    if start_angle == pouring_angle[0]+12:
      rospy.sleep(2)
    if start_angle == pouring_angle[0]+15:
      rospy.sleep(1)
    if start_angle > pouring_angle[0]+15:
      rospy.sleep(0.3)
    if start_angle > pouring_angle[0]+23+6:
      #rospy.sleep(3)
      raw_input()
      if start_angle == pouring_angle[0]+23+6 :
        moveTo(group,'y',group.get_current_pose().pose.position.y-0.02) 
      if start_angle == pouring_angle[0]+23+6 :
        moveTo(group,'y',group.get_current_pose().pose.position.y-0.02) 
      if start_angle == pouring_angle[0]+21+12 :
        moveTo(group,'y',group.get_current_pose().pose.position.y+0.04) 


  i=0
  group.set_max_acceleration_scaling_factor(1)
  while i<9 :
    i+=1
    moveTo(group,'z',group.get_current_pose().pose.position.z+0.1) 
    moveTo(group,'z',group.get_current_pose().pose.position.z-0.1) 
    if i==2 or i==5 or i==8 :
      raw_input()
  group.set_max_acceleration_scaling_factor(0.5)

  back_pose=group.get_current_pose()
  back_pose.pose.orientation =original_orientation
  group.set_max_velocity_scaling_factor(0.2)
  group.go(back_pose)
  
def GoAndPour(group) :
  moveTo(group,'z',0.6)   # try to delete this motion
  moveTo(group,'y',(group.get_current_pose().pose.position.y+startPouring[1])/2) 
  moveTo(group,'x',startPouring[0])  
  moveTo(group,'y',startPouring[1]) 
  moveTo(group,'z',(group.get_current_pose().pose.position.z+startPouring[2])/2) 
  moveTo(group,'z',startPouring[2]) 
 
  pouring(group)
  
  
def deliverCoffeeToCustmoer(group,pub,command,Pose):
  X=Pose[0]
  Y=Pose[1]
  Z=Pose[2]
  if abs(group.get_current_pose().pose.position.x-corridorX)>0.3:
    moveTo(group,'mx',(group.get_current_pose().pose.position.x+corridorX)/2)
  moveTo(group,'mx',corridorX)
  moveTo(group,'mz',(group.get_current_pose().pose.position.z+corridorZ)/2)
  moveTo(group,'mz',corridorZ)
  if abs(group.get_current_pose().pose.position.y-Y)>0.4:
    moveTo(group,'my',(group.get_current_pose().pose.position.y+Y)/3)
  moveTo(group,'my',(group.get_current_pose().pose.position.y+Y)/2)
  moveTo(group,'my',Y)
  moveTo(group,'mz',(group.get_current_pose().pose.position.z+Z)/2)
  moveTo(group,'mz',Z)
  if abs(group.get_current_pose().pose.position.x-X)>0.3:
    moveTo(group,'mx',(group.get_current_pose().pose.position.x+X)/2)
  moveTo(group,'mx',X)
 
  moveTo(group,'sz',Pose[2]-0.006)
  #raw_input()  ##convenient to correct pose @Sampson 
  gposition(pub,command,Pose[3])
  rospy.sleep(2)
  moveTo(group,'sx',Pose[0]+0.07)


                               
################################ Main sequence of program ##########################

  
def make_latte_art(group,pub,command,latteArtPattern) :
#####Change pattern here
  print "make latte art"
  if latteArtPattern=="1":
    print "Heart Shape is still developing."
  if latteArtPattern=="2":
    drawSingleLeaf(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  if latteArtPattern=="4":
    print "Flower is selected!"
    drawFlower(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  else:
    print "No such pattern!"
  #drawSwirl(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,6,0.03,1)
  #drawFlower(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  #drawCircle(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04,3,1) ## shake the milk
  drawSingleLeaf(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.03)
  #drawHalfLeaf(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  #drawRose(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  #drawLuckyLeaf(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  ##drawHeart(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)

def startMainSequence(group,pub,command,latteArtPattern) :
  print "startMainSequence?"
  raw_input()

## press milk machine button
  command=presetToSafePosition(group,pub)
  PressMilkButton(group,pub,command,milkButton)

## move cup to machine and make coffee
  RotateAboutAxisBy(group,'y',35,-1)
  PickUpAt(group,pub,command,cupInitial)
  PlaceAt(group,pub,command,cupAtCoffeeMachine)
  gposition(pub,command,255)                   ##close gripper before pressing button
  PlaceAt(group,pub,command,button)
  PickUpAt(group,pub,command,cupAtCoffeeMachine)
  PlaceAt(group,pub,command,cupInProcess)
  moveTo(group,'z',0.6)                    ##otherwise hit the cup
  RotateAboutAxisBy(group,'y',35,1)

## pick and pour milk
  grabMilkStir(group,pub,command)
  PickUpMilkAt(group,pub,command,milkInitial)
  GoAndPour(group)
  PlaceAt(group,pub,command,milkFinal)
  

## make latte art
  PickUpStick(group,pub,command)
  ## Latte Art
  print "Start To draw latte art"
  make_latte_art(group,pub,command,latteArtPattern)
  PlaceStickAt(group,pub,command)
  
## give coffee to customer

  command.rFR  = 200
  moveTo(group,'z',0.4)
  RotateAboutAxisBy(group,'y',35,-1)	
  PickUpAt(group,pub,command,cupInProcess)
  deliverCoffeeToCustmoer(group,pub,command,cupFinal)
  command.rFR  = 150



def repeatRun(latteArtPattern):
  while True:
    receiveOrder(latteArtPattern)
    command=presetToSafePosition(group,pub)
    talker_ready()


def fullRun(group,pub,command):
  latteArtPattern="0"
  latteArtPattern=receiveOrder(latteArtPattern)
  inputCupPositionFromFile(CupPositionFile)
  startMainSequence(group,pub,command,latteArtPattern)
  talker_ready()


def test(group,pub,command) :
  print "test"
    
  #fullRun(group,pub,command)

  
  inputCupPositionFromFile(CupPositionFile)
  command=presetToSafePosition(group,pub)
  PickUpAt(group,pub,command,cupInitial)

  #group.set_max_velocity_scaling_factor(0.05)
  #command=presetToSafePosition(group,pub)
  #command=gactive(pub)
  #gposition(pub,command,170)          
  #startMainSequence(group,pub,command)
  
#  moveTo(group,'z',0.6)   # try to delete this motion
#  moveTo(group,'y',(group.get_current_pose().pose.position.y+startPouring[1])/2) 
#  moveTo(group,'x',startPouring[0])  
#  moveTo(group,'y',startPouring[1]) 
#  moveTo(group,'z',(group.get_current_pose().pose.position.z+startPouring[2])/2) 
#  moveTo(group,'z',startPouring[2]) 
  ##freefallpour(group)
  ##latte_art_test(group,pub,command)
  ##drawSingleLeaf(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04) 
  ##drawHeart(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  #grabMilkStir(group,pub,command)
  #PickUpMilkAt(group,pub,command,milkInitial)
  #GoAndPour(group)
  #PlaceAt(group,pub,command,milkFinal)
  #PickUpStick(group,pub,command)
  #PlaceStickAt(group,pub,command)
  ##drawHeart(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)
  #grabMilkStir(group,pub,command)
  
  #RotateAboutAxisBy(group,'y',35,-1)
  #PickUpAt(group,pub,command,cupInitial)
  #SafeMoveToXYZ(group,corridorX,cupAtCoffeeMachine[1],corridorZ)
  #PlaceAt(group,pub,command,cupInProcess)
  ##RotateAboutAxisBy(group,'y',35,1)


  #drawLuckyLeaf(group,group.get_current_pose().pose.position.x,group.get_current_pose().pose.position.y,0.04)

  
  #latteArtPattern="0"
  #latteArtPattern=receiveOrder(latteArtPattern)
  
  #make_latte_art(group,pub,command,5)

  #talker_ready()
  
                               
################################ Setting of the program #############################
def CoffeeMakingByXYZ():
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

  temporary.append(group.get_current_pose().pose)
  

  
  group.set_max_acceleration_scaling_factor(0.5)
 
  test(group,pub,command)
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
    CoffeeMakingByXYZ()
  except rospy.ROSInterruptException:
    pass

