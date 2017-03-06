#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import sys
import time

from geometry_msgs.msg import Pose

from hlpr_speech_synthesis import speech_synthesizer 
from hlpr_speech_recognition import speech_listener
from hlpr_manipulation_utils import manipulator
from hlpr_manipulation_utils.manipulator import *
from hlpr_manipulation_utils.arm_moveit import *

from modules.utilities.baris_utils import *

_manipulator = None
_arm_planner = None
_speech_synth = None
_isTheManipulationStateGlobalsInitialized = False

#see if it makes sense to pass these with userdata
def initGlobals():
  print 'Initializing manipulation state globals'

  global _manipulator
  global _arm_planner
  global _speech_synth
  global _isTheManipulationStateGlobalsInitialized

  _arm_planner = ArmMoveIt()
  _manipulator = Manipulator()
  _speech_synth = speech_synthesizer.SpeechSynthesizer()
  _isTheManipulationStateGlobalsInitialized = True


def sendPlan(arm,plannedTra):
  traj_goal = FollowJointTrajectoryGoal()

  traj_goal.trajectory = plannedTra.joint_trajectory
  arm.smooth_joint_trajectory_client.send_goal(traj_goal)#sendWaypointTrajectory(traj_goal)
  arm.smooth_joint_trajectory_client.wait_for_result()   
  return arm.smooth_joint_trajectory_client.get_result() 

class ManipulateObjectMainState(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['moveArm', 'gripper', 'findObject', 'succeeded', 'failed', 'aborted','planPath', 'pickRetract'],
                         input_keys=['armGoalIn', 'graspResultIn', 'objectLocationIn', 'pathFoundIn', 'pickResultIn','targetPoseIn'], 
                         output_keys=['traOut','gripperCommandOut','statusOut'])
    self.status = 'begin'

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    self.ss = _speech_synth

  def execute(self, userdata):
    userdata.statusOut = None
    rospy.loginfo('Manipulate object substate machine')

    if self.status is 'begin':
      self.status = 'findObject'
      return 'findObject'

    elif self.status is 'findObject':
      if userdata.objectLocationIn is None:
        print 'Failed because no object found'
        userdata.statusOut = 'failed'
        return 'failed'
      self.status = 'planning'
      return 'planPath'

    elif self.status is 'planning':
      if not userdata.pathFoundIn:
        print 'Failed because no plan found'
        userdata.statusOut = 'failed'
        return 'failed'
      self.status = 'preGrasp'
      return 'moveArm'

    elif self.status is 'preGrasp':
      if userdata.targetPoseIn is None:
        print 'Failed because could not execute'
        userdata.statusOut = 'failed'
        return 'failed'
      else:
        self.status = 'pickRetract'
        return 'pickRetract'

      #userdata.gripperCommandOut = 'close'
      #userdata.traOut = None
      #self.status = 'grasp'

    elif self.status is 'pickRetract':
      if userdata.pickResultIn:
        self.status = 'success'
        return 'succeeded'
      else:
        return 'failed'
    else:
      print 'How the hell did I manage to come here! Status: ' + str(self.status)
      self.ss.say('Human I am not in a correct state, how did I get here?')
      time.sleep(1)
      return 'failed'  
  
    #elif self.status is 'grasp':
    #  if userdata.graspResultIn:
    #     userdata.gripperCommandOut = None
    #     userdata.traOut = 'retract'
    #     self.status = 'retract'
    #  else:
    #     userdata.gripperCommandOut = None
    #     userdata.traOut = 'retract'
    #     #self.status =           
    
class ManipulateObjectState(smach.State):
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

class ExecuteTrajectoryState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['trajectoryIn'],
                         output_keys=['execResultOut'])


    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    self.arm = _manipulator.arm # Arm()
    self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()

    #in the future these will be in a database
    handOffTra = [[-1.90, 1.50, 0.50, -2.00, 3.00, 0.72],
                  [-1.80, 1.80, 1.00, -2.10, 2.50, 0.72],
                  [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90],
                  [-1.60, 2.20, 0.80, -2.20, 1.50, 1.20],
                  [-1.60, 2.40, 1.00, -2.50, 1.50, 1.20],
                  [-1.60, 2.60, 1.20, -3.14, 1.50, 1.20]]

    self.traDict = {'person':handOffTra}
    self.functionDict = {'retract':self.arm.upper_tuck}

  def execute(self, userdata):
    rospy.loginfo('Moving the arm')
    userdata.execResultOut = None
    if userdata.trajectoryIn is None:
      print 'Received a None trajectory, this should not have happened'
      return 'failed'
    print 'Executing trajectory'
    self.ss.say("Now executing the arm trajectory")

    if not isinstance(userdata.trajectoryIn, str):
      if isinstance(userdata.trajectoryIn, list):
        self.arm.sendWaypointTrajectory(userdata.trajectoryIn)
      else:
        if len(userdata.trajectoryIn.joint_trajectory.points) < 1:
          return 'failed'
        sendPlan(self.arm, userdata.trajectoryIn)  
    else:
      try:
        self.functionDict[userdata.trajectoryIn]()
      except KeyError:
        try:
          self.arm.sendWaypointTrajectory(self.traDict[userdata.trajectoryIn])
        except KeyError:
          print 'Do not know how to execute ' + userdata.trajectoryIn
          return 'failed'
    userdata.execResultOut = 'done'
    return 'succeeded'

class UseGripperState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['gripperCommandIn','waitForSpeech'],
                         output_keys = ['resultOut'])

    self.sl = speech_listener.SpeechListener()
    self.gripper_command = None

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
  
    self.gripper = _manipulator.gripper

  def execute(self, userdata):
    rospy.loginfo('Using the gripper, how exciting!')
    if userdata.waitForSpeech:
      while not rospy.is_shutdown():
        if self.sl.get_last_command() == 'OPEN_HAND':
          self.gripper_command = 'open'
          break
        if self.sl.get_last_command() == 'CLOSE_HAND':
          self.gripper_command = 'close'
          break
        if self.sl.get_last_command() == 'END':
          return 'aborted'
        time.sleep(0.2)
    else:
      self.gripper_command = userdata.gripperCommandIn

    print 'Executing command ' + self.gripper_command
    if self.gripper_command == 'open':
      self.gripper.open()
    elif self.gripper_command == 'close':
      self.gripper.close()

    for i in range(0,2):
      sys.stdout.write('-')
      sys.stdout.flush()
      time.sleep(0.5)
    print '> Done!'
    userdata.resultOut = True
    time.sleep(0.5)
    return 'succeeded'

class PlanTrajectoryState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['objectLocationIn'],
                         output_keys=['traOut','pathFoundOut','targetPoseOut'])

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()

    self.arm_planner = _arm_planner
    self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()

    self.targetPose = None

  def execute(self, userdata):
    rospy.loginfo('Calculating path')
    userdata.pathFoundOut = False
    userdata.traOut = None
    userdata.targetPoseOut = None

    print 'Creating the trajectory to pose ' + str(userdata.objectLocationIn)
    self.ss.say('Creating the trajectory')# to pose ' + str(userdata.objectLocationIn))

    self.targetPose = transform2pose(userdata.objectLocationIn)

    if self.targetPose is None:
      print 'Received None pose, this should not have happened'
      self.ss.say('Invalid pose received')
      return 'failed'

    #self.targetPose.position.z += 0.25
    self.targetPose.position.z += 0.10 #TODO: verify on the real robot (I think this takes into account the old EEF offset) which was 0.13 m
    self.targetPose.position.x += 0.040
    orient = Quaternion()
    orient.x = 0.711924914599
    orient.y = -0.701746728872
    orient.z = -0.0256686053995
    orient.w = 0.00745434012957
    self.targetPose.orientation = orient #quatFromAngleAxis([0.5094,0.5094,0.5094]) 

    self.arm_planner.group[0].set_pose_reference_frame('base_link')
    plannedTra = self.arm_planner.plan_poseTargetInput(self.targetPose)
    #joints = self.arm_planner.get_IK(self.targetPose)
    #plannedTra = self.arm_planner.plan_jointTargetInput(joints)

    if plannedTra is None:
      print 'Could not find a plan'
      self.ss.say('Could not find a plan')
      time.sleep(0.5)
      return 'failed'

    self.ss.say("done!")
    time.sleep(0.5)
    userdata.traOut = plannedTra
    userdata.pathFoundOut = True
    userdata.targetPoseOut = self.targetPose
    return 'succeeded'

class CompositePickAndRetractState(smach.State):
  def __init__(self, ik_root = 'base_link'):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['initArmPoseIn'],
                         output_keys=['pickResultOut'])

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()

    self.ss = _speech_synth
    self.manip = _manipulator
    self.arm_planner = _arm_planner
    self.ik_root = ik_root

  def execute(self, userdata):
    rospy.loginfo('Picking and retracting')
    
    userdata.pickResultOut = False

    if userdata.initArmPoseIn is None:
      print 'Received a None pose, this should not have happened'
      return 'failed'

    curPose = rosPoseCopy(userdata.initArmPoseIn)
    targetPose = rosPoseCopy(userdata.initArmPoseIn)
    targetPose.position.z = max(0.91, targetPose.position.z-0.10)
    poses = straightLinePoses(curPose, targetPose)
    jntWps = self.arm_planner.wayPointIK(poses, 3, self.ik_root)
    if jntWps is None:
      print 'Could not calculate IK'
      self.ss.say('Could not find inverse kinematics solution')
      return 'failed'
    self.manip.arm.sendWaypointTrajectory(jntWps)

    time.sleep(0.5)
    self.manip.gripper.close()
    time.sleep(1.5)

    curPose = rosPoseCopy(targetPose)
    targetPose.position.z += 0.12
    poses = straightLinePoses(curPose, targetPose)
    jntWps = self.arm_planner.wayPointIK(poses, 3, self.ik_root)
    if jntWps is None:
      print 'Could not calculate IK'
      self.ss.say('Could not find inverse kinematics solution')
      return 'failed'
    self.manip.arm.sendWaypointTrajectory(jntWps)
    
    ut_wp = [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90]
    plannedTra = self.arm_planner.plan_jointTargetInput(ut_wp)
    if plannedTra is None:
      print 'Could not find a plan to retract the arm'
      self.ss.say('Could not find a plan to retract the arm')
      time.sleep(0.5)
      return 'failed'

    sendPlan(self.manip.arm, plannedTra)
    self.manip.arm.upper_tuck()

    userdata.pickResultOut = True
    return 'succeeded'

