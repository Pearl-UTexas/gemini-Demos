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
from hlpr_speech_synthesis import speech_synthesizer 
import modules.navigation_module as NM

class GoToPointState(smach.State):
  def __init__(self,  withRobot = True):
    smach.State.__init__(self,
                         outcomes=['succeeded','aborted'],
                         input_keys=['navGoalIn'])
    self.counter = 0
    self.navGoal = NM.NavigationGoal(withRobot)
    
    #TODO: The following co-ordinates of elevator and other have not been tested on the real robot. Subject to changes.
    self.elevator = Pose()
    self.elevator.position.x = 1.4      #1.4, 23.959  
    self.elevator.position.y = 1.90      #1.95, -5.222 
    self.elevator.position.z = 0.0

    self.elevator.orientation.x = 0.000
    self.elevator.orientation.y = 0.000
    self.elevator.orientation.z = 0.0    #-0.684 
    self.elevator.orientation.w = 1.0    #0.730

    self.other = Pose()
    self.other.position.x = 2.16          #2.480 
    self.other.position.y = -1.67          #-3.679 
    self.other.position.z = 0.0

    self.other.orientation.x = 0.000
    self.other.orientation.y = 0.000
    self.other.orientation.z = -0.198       #1.00 
    self.other.orientation.w = 0.98       #0.023 

    self.poseDict = {'elevator':self.elevator,'person':self.other}

    self.ss = speech_synthesizer.SpeechSynthesizer()


  def execute(self, userdata):
    rospy.loginfo('Navigating to a point')    
    self.ss.say("Now I am navigating to the " + userdata.navGoalIn)
    self.navGoal.setGoalPose(self.poseDict[userdata.navGoalIn])
    time.sleep(0.5)
    
    return 'succeeded'
