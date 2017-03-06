# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 22:49:56 2016

@author: baris
"""
import smach
import rospy
import sys
import time
import smach_ros

from geometry_msgs.msg import *
from std_msgs.msg import *
from espeak import espeak

from base_nav_goal import GoalMaker
from pan_tilt import PT
from pose_stuff import PS
from ptps import PTPS

#These will probably be action states. These can potentially be ran concurrently with some of the other states. 
#The idea is these can be sued as building blocks, they won't get changed too much

class ExecTra(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['trajectoryIn'],
                         output_keys=['execResultOut'])

  def execute(self, userdata):
    rospy.loginfo('Moving the arm')
    print 'Executing trajectory ' + userdata.trajectoryIn
    espeak.synth("Now executing the arm trajectory")
    for i in range(0,5):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
    print '> Good!'
    time.sleep(0.5)
    return 'succeeded'
    
class UseGripper(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['gripperCommandIn'],
                         output_keys = ['resultOut'])   

  def execute(self, userdata):
    rospy.loginfo('Moving the arm')
    print 'Executing command ' + userdata.gripperCommandIn
    for i in range(0,2):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
    print '> Done!'
    userdata.resultOut = True
    time.sleep(0.5)
    return 'succeeded'
    
    
class GoToPoint(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','aborted'],
                         input_keys=['navGoalIn'])
    self.counter = 0
    self.gm = GoalMaker()

    self.table = Pose()
    self.table.position.x = 3.32 #3.7 #3.1489
    self.table.position.y = -0.23 #0.6 #1.091
    self.table.position.z = 0.0

    self.table.orientation.x = 0.000
    self.table.orientation.y = 0.000
    self.table.orientation.z = 0.82 #0.7071 #0.75681
    self.table.orientation.w = 0.57 #0.7071 #0.65364

    self.other = Pose()
    self.other.position.x = 2.56 #0.7#0.8636
    self.other.position.y = -3.23 #0.7#1.2316
    self.other.position.z = 0.0

    self.other.orientation.x = 0.000
    self.other.orientation.y = 0.000
    self.other.orientation.z = 0.27 #0.97438#0.99992
    self.other.orientation.w = 0.96 #0.22492#-0.012469

    self.poseDict = {'table':self.table,'person':self.other}


  def execute(self, userdata):
    rospy.loginfo('Navigating to a point')    
    #print 'Going to the ' + userdata.navGoalIn
    #for i in range(0,11):
    #  sys.stdout.write('-')
    #  sys.stdout.flush()
    #  time.sleep(0.5)
    #print '> Done! Reached the ' + userdata.navGoalIn
    espeak.synth("Now I am navigating to the " + userdata.navGoalIn)
    self.gm.move(self.poseDict[userdata.navGoalIn])
    time.sleep(0.5)
    
    return 'succeeded'
    
    
class FindObject(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['succeeded','failed','aborted'],
                         input_keys = ['objectIn'],
                         output_keys = ['objectLocationOut'])

    self.ptps = PTPS()

  def execute(self, userdata):
    rospy.loginfo('Executing state FindObject')
    print 'Scan initiated to find the ' + userdata.objectIn
    espeak.synth("Scan initiated to find the" + userdata.objectIn)
    #for i in range(0,11):
    #  sys.stdout.write('-')
    #  sys.stdout.flush()
    #  time.sleep(0.5)
    #userdata.objectLocationOut = [0,0,0,0,0,0,1]
    #print '> Target Found! The ' + userdata.objectIn + ' at location ' +  str([0,0,0,0,0,0,1])

    if(self.ptps.objectSearch()):
      self.ptps.directLookAtObject()
      espeak.synth('Object found')
    else:
      espeak.synth('no object found')
    time.sleep(0.5)
    self.ptps.pt.nod()
    time.sleep(0.5)
    self.ptps.pt.bothMore([0,0],10,10)
    self.ptps.ps._updateTransform()
    userdata.objectLocationOut = self.ptps.ps.tr
    print self.ptps.ps.tr

    return 'succeeded'  
  
class PlanPath(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['objectLocationIn'],
                         output_keys=['traOut'])

  def execute(self, userdata):
    rospy.loginfo('Calculating path')
    print 'Creating the trajectory to pose ' + str(userdata.objectLocationIn)
    espeak.synth('Creating the trajectory')# to pose ' + str(userdata.objectLocationIn))
    for i in range(0,5):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
    print '> Done!'
    espeak.synth("done!")
    time.sleep(0.5)
    userdata.traOut = 'IK_Tra'
    return 'succeeded'
