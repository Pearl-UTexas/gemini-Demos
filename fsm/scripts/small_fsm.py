#!/usr/bin/env python

import roslib; #roslib.load_manifest('hello_world')
import rospy
import smach
import smach_ros

import sys
import time

from SmallStates import *
from espeak import espeak

from speech_recog import speech_listener

#comments:
#execute might take all the cycles, not very c6 like, see if smach has an updateOnce/executeOnce option

class Idle(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         #outcomes=['move','use_arm','use_arm_sub','find_object','idling','done'],
                         outcomes=['move','use_arm','use_arm_sub','idling','done'],
                         input_keys=['idleStartCommandIn'],
                         output_keys=['idleNavGoalOut', 'idleArmGoalOut'] )
    self.counter = 0
    self.transitionList = ['move','use_arm_sub','move','use_arm','done']
    self.transitionCount = -1
    self.sl = speech_listener()
    self.receivedStartCommand = False

    espeak.synth("I am awake, I am awake")

  def execute(self, userdata):
    speech_com = self.sl.get_last_command()
    
    if speech_com == 'START':
      self.receivedStartCommand = True
      espeak.synth("Here we go")
    elif speech_com == 'GREETING':
      espeak.synth("Hello Human")
    elif speech_com == 'SMALL_TALK':
      espeak.synth("Excited as always")
    elif speech_com == 'HEAR_CHECK':
      espeak.synth("Yes, can you hear me?")
    

    if not (userdata.idleStartCommandIn and self.receivedStartCommand):
        #rospy.loginfo('I am waiting on you')
        time.sleep(0.5)
        return 'idling'
    else:
        rospy.loginfo('Executing state Idle')
        self.transitionCount += 1
        if self.transitionCount == 0:
            userdata.idleNavGoalOut = 'table'
        elif self.transitionCount == 2:
            userdata.idleNavGoalOut = 'person'
        else:
            userdata.idleNavGoalOut = None
        if self.transitionCount == 1:
            userdata.idleArmGoalOut = 'object'
        elif self.transitionCount == 3:
            userdata.idleArmGoalOut = 'person'
        else:
            userdata.idleArmGoalOut = None
        if(self.transitionList[self.transitionCount] == 'done'):
            espeak.synth("I am done. Have a nice day")
            time.sleep(1);
        return self.transitionList[self.transitionCount]
    
class ManipulateObjectMain(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['moveArm', 'gripper', 'findObject', 'succeeded', 'aborted','planPath'],
                         input_keys=['armGoalIn', 'graspResultIn', 'objectFoundIn', 'pathFoundIn'], 
                         output_keys=['traOut','gripperCommandOut'])
    self.status = 'begin'

  def execute(self, userdata):
    rospy.loginfo('Manipulate object substate machine')
    if userdata.armGoalIn is 'object':
        act = 'grasping'
    else:
        act = 'handing over'
    if self.status is 'begin':
        self.status = 'objectSearch'
        return 'findObject'
    elif self.status is 'findObject':
      if userdata.objectFoundIn:
        userdata.gripperCommandOut = None
        userdata.traOut = 'preGrasp'
      else:
        return 'aborted'
      self.status = 'preGrasp'
      return 'moveArm'
    elif self.status is 'preGrasp':
      userdata.gripperCommandOut = 'ForceGrasp'
      userdata.traOut = None
      self.status = 'grasp'
    elif self.status is 'grasp':
      if userdata.graspResultIn:
         userdata.gripperCommandOut = None
         userdata.traOut = 'retract'
         self.status = 'retract'
      else:
         userdata.gripperCommandOut = None
         userdata.traOut = 'retract'
         #self.status =           
   
    return 'succeeded'
    
class ManipulateObject(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['succeeded','aborted'],
                         input_keys=['armGoalIn'])

  def execute(self, userdata):
    rospy.loginfo('Moving the arm')
    if userdata.armGoalIn is 'object':
        act = 'grasping'
    else:
        act = 'handing over'
    for i in range(0,11):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
      if i is 5:
        sys.stdout.write('- ' + act + ' -')
        sys.stdout.flush()
        time.sleep(2)
    print '> Good!'
    time.sleep(0.5)
    return 'succeeded'
# main
def main():
  rospy.init_node('smach_example_state_machine')

  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['end'])
  sm.userdata.startCommand = False
  sm.userdata.object = 'speaker'
  
  sm.userdata.navGoal = None
  sm.userdata.armGoal = None
  sm.userdata.tra = 'handOff'

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('Idle', Idle(), 
                            transitions={'idling':'Idle', 
                                         'move':'GoToPoint',
                                         'use_arm':'ExecTra',
                                         'use_arm_sub':'manip_sub',
                                         'done':'end'},
                            remapping={'idleStartCommandIn':'startCommand', 
                                       'idleNavGoalOut':'navGoal',
                                       'idleArmGoalOut':'armGoal'})

    smach.StateMachine.add('GoToPoint', GoToPoint(), 
                            transitions={'succeeded':'Idle', 
                                         'aborted':'end'},
                            remapping={'navGoalIn':'navGoal'})
                            
    smach.StateMachine.add('ExecTra', ExecTra(), 
                              transitions={'succeeded':'Idle', 
                                           'failed':'end',
                                           'aborted':'end'},
                              remapping={'trajectoryIn':'armGoal'})
                                         
                                                    
    manip_sub = smach.StateMachine(outcomes=['succeededMO','abortedMO'],
                                   input_keys = ['armGoalMO','objectMO'])
  
    manip_sub.userdata.armGoalMO = None
    manip_sub.userdata.objectMO = sm.userdata.object
    manip_sub.userdata.tra = None
    manip_sub.userdata.objectLocation = None
    manip_sub.userdata.gripperCommand = None
    
#Get the arm goal into the sub state    

#mixing and matching sequential transitions and 1 main state type structures
    with manip_sub:
      smach.StateMachine.add('ManipulateObjectMain', ManipulateObjectMain(), 
                              transitions={'findObject':'FindObject',
                                           'planPath':'PlanPath',
                                           'moveArm':'ExecTra', 
                                           'gripper':'UseGripper',
                                           'succeeded':'succeededMO',
                                           'aborted':'abortedMO'},
                              remapping={'armGoalIn':'armGoalMO',
                                         'objectLocationIn':'objectLocation',
                                         'gripperCommandOut':'gripperCommand',
                                         'traOut':'tra'})
                                         
      smach.StateMachine.add('FindObject', FindObject(), 
                              transitions={'succeeded':'PlanPath', 
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'objectIn':'objectMO',
                                         'objectLocationOut':'objectLocation'})
                                         
      smach.StateMachine.add('PlanPath', PlanPath(), 
                              transitions={'succeeded':'ExecTra', 
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'objectLocationIn':'objectLocation',
                                         'traOut':'tra'})
                              
      smach.StateMachine.add('ExecTra', ExecTra(), 
                              transitions={'succeeded':'ManipulateObjectMain', 
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'trajectoryIn':'tra'})
                             
      smach.StateMachine.add('UseGripper', UseGripper(), 
                              transitions={'succeeded':'ManipulateObjectMain',
                                           'failed':'ManipulateObjectMain',
                                           'aborted':'abortedMO'},
                              remapping={'gripperCommandIn':'gripperCommand',
                                         'resultOut':'graspResult'})
                              
    smach.StateMachine.add('manip_sub', manip_sub,
                            transitions={'succeededMO':'Idle', 
                                         'abortedMO':'end'},
                            remapping={'armGoalMO':'armGoal',
                                       'objectMO':'object'})
                                       
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  # Execute SMACH plan
  outcome = sm.execute()
  print outcome
#  rospy.spin()
  sis.stop()


if __name__ == '__main__':
  main()


#  def execute(self, userdata):
#    rospy.loginfo('Manipulate object substate machine')
#    if userdata.armGoalIn is 'object':
#        act = 'grasping'
#    else:
#        act = 'handing over'
#    if self.status is 'begin':
#        self.status = 'objectSearch'
#        return 'findObject'
#    elif self.status is 'findObject':
#      if userdata.objectFoundIn:
#        userdata.gripperCommandOut = None
#        userdata.traOut = 'preGrasp'
#      else:
#        return 'aborted'
#     self.status = 'preGrasp'
#     return 'moveArm'
#   elif self.status is 'preGrasp':
#     userdata.gripperCommandOut = 'ForceGrasp'
#     userdata.traOut = None
#     self.status = 'grasp'
#   elif self.status is 'grasp':
#     if userdata.graspResultIn:
#        userdata.gripperCommandOut = None
#        userdata.traOut = 'retract'
#        self.status = 'retract'
#     else:
#        userdata.gripperCommandOut = None
#         userdata.traOut = 'retract'
#         #self.status =           
#   
#    return 'succeeded'
