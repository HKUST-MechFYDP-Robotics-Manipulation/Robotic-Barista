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
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os,sys
import subprocess
import glob
import json
from os import path
## END_SUB_TUTORIAL
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from apriltags_ros.msg import AprilTagDetectionArray
from pprint import pprint
##preset
import numpy
import math
scale=1
if __name__=='__main__':
  rospy.init_node('aaa',
                  anonymous=True)
  l=input('x')
  k=input('y')
  m=input('z')
  while True:
    msg = rospy.wait_for_message('/tag_detectionss',AprilTagDetectionArray)
    print 'not find'
    print msg
    print len(msg.detections)
    ##if msg.detections!=[]:
    if (len(msg.detections)==2):
      break
  for a in range (0,len(msg.detections)):
    if (msg.detections[a].id==0):
       b=a
       #break
    if (msg.detections[a].id==2):
       c=a
       #break
  bq=[1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0]
  cq=[1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0]
  dq=[1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0]
  eq=[1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0]
  bq[0]=msg.detections[b].id
  bq[1]=msg.detections[b].pose.pose.position.x/scale
  bq[2]=msg.detections[b].pose.pose.position.y/scale
  bq[3]=msg.detections[b].pose.pose.position.z/scale
  bq[4]=msg.detections[b].pose.pose.orientation.x
  bq[5]=msg.detections[b].pose.pose.orientation.y
  bq[6]=msg.detections[b].pose.pose.orientation.z
  bq[7]=msg.detections[b].pose.pose.orientation.w
  print bq
  dist=math.sqrt(bq[1]**2+bq[2]**2+bq[3]**2)
  print 'dist'
  print dist
  
  br=numpy.array([[1.0-2*bq[5]**2-2*bq[6]**2,2.0*(bq[4]*bq[5]-bq[7]*bq[6]),2.0*(bq[7]*bq[5]+bq[4]*bq[6]),bq[1]],[2.0*(bq[7]*bq[6]+bq[4]*bq[5]),1.0-2.0*bq[4]**2-2*bq[6]**2,2.0*(bq[6]*bq[5]+bq[4]*bq[7]),bq[2]],[2.0*(bq[4]*bq[6]+bq[7]*bq[5]),2.0*(bq[7]*bq[4]+bq[5]*bq[6]),1.0-2.0*bq[4]**2-2*bq[6]**2,bq[3]],[0,0,0,1]])

  cq[0]=msg.detections[c].id
  cq[1]=msg.detections[c].pose.pose.position.x/scale
  cq[2]=msg.detections[c].pose.pose.position.y/scale
  cq[3]=msg.detections[c].pose.pose.position.z/scale
  cq[4]=msg.detections[c].pose.pose.orientation.x
  cq[5]=msg.detections[c].pose.pose.orientation.y
  cq[6]=msg.detections[c].pose.pose.orientation.z
  cq[7]=msg.detections[c].pose.pose.orientation.w
  cr=numpy.array([[1.0-2*cq[5]**2-2*cq[6]**2,2.0*(cq[4]*cq[5]-cq[7]*cq[6]),2.0*(cq[7]*cq[5]+cq[4]*cq[6]),cq[1]],[2.0*(cq[7]*cq[6]+cq[4]*cq[5]),1.0-2.0*cq[4]**2-2*cq[6]**2,2.0*(cq[6]*cq[5]+cq[4]*cq[7]),cq[2]],[2.0*(cq[4]*cq[6]+cq[7]*cq[5]),2.0*(cq[7]*cq[4]+cq[5]*cq[6]),1.0-2.0*cq[4]**2-2*cq[6]**2,cq[3]],[0,0,0,1]])

  Tr=numpy.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
  brinv=numpy.linalg.inv(br)
  cam=numpy.matmul(brinv, Tr)
  base=numpy.matmul(br, cam)
  tag2=numpy.matmul(cam, cr)
  dis=math.sqrt(cam[0][3]**2+cam[1][3]**2+cam[2][3]**2)
  print 'cam'
  print cam
  print 'dis'
  print dis
  print 'base'
  print base
  print'tag2'
  print tag2
  print math.sqrt(tag2[0][3]**2+tag2[1][3]**2+tag2[2][3]**2)
  print (0.1-math.sqrt(tag2[0][3]**2+tag2[1][3]**2+tag2[2][3]**2))/0.1*100
  mat2=[1.0,2.0,3.0,4.0]
  f = open('test.txt', 'w')
  print len(msg.detections)
  r=tag2[0][3]
  s=tag2[1][3]
  t=tag2[2][3]
  print "r"
  print -(s+k)
  print r+l
  print t+m
  f.write(str(-(s+k)))
  f.write(' \n')
  f.write(str(r+l))
  f.write(' \n')
  f.write(str(t+m))
  #f = open('20279360.txt','w')
  #f.write(msg.poses[0].position.x)
  #f.write(msg.poses[0].position.y)
  #f.write(msg.poses[0].position.z)
  #f.write(msg.poses[0].orientation.x)
  #f.write(msg.poses[0].orientation.y)
  #f.write(msg.poses[0].orientation.z)
  #f.write(msg.poses[0].orientation.w)""""
  ##message = f.read()
  ##print(message)
  f.close()

