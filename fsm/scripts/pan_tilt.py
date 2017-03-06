#!/bin/env python
import roslib; 
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import rospy
import time
from math import *

#returns the plausible root of sin(x)*c1+cos(x)*c2 = c3
def trigSolution(params):
  y = atan(abs(params[1]/params[0]))
  h = params[2]/sqrt(params[0]*params[0]+params[1]*params[1])
  if(params[0] > 0 and params[1] < 0):
    return asin(h) + y
  if(params[0] < 0 and params[1] < 0):
    return asin(-h) - y
  if(params[0] < 0 and params[1] > 0):
    return asin(-h) + y
  else:
    return asin(h) - y

#TODO: decouple IK functionality from ROS stuff

class PT:
  def __init__(self):
    self.pubTilt = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
    self.pubPan  = rospy.Publisher('/pan_controller/command', Float64, queue_size=10)

    self.subTilt = rospy.Subscriber("/tilt_controller/state", JointState, self.cb_tilt)
    self.subPan = rospy.Subscriber("/pan_controller/state", JointState, self.cb_pan)

    self.tilt_pos = 0.0
    self.pan_pos = 0.0
#y1,x2,y2,z2
    self.pt_params = [-0.0985, (0.03325+0.06175)/2, -0.04201 ,0.0245]

    #whether to calculate the joint position for the imaginary link at the end for ik
    self.calc_ik_im_trans = False

  def tilt(self, pos):
    tmp = Float64()
    tmp.data = pos
    self.pubTilt.publish(tmp)
    
  def pan(self, pos):
    self.pubPan.publish(pos)


  def both(self, pos):
    self.pubPan.publish(pos[0])
    self.pubTilt.publish(pos[1])

  def bothMore(self, pos, repetitions = 10, rate = 10):
    ros_rate = rospy.Rate(rate)
    for i in range(0,repetitions):
      self.both(pos)
      ros_rate.sleep()

  def cb_tilt(self, js):
    self.tilt_pos = js.current_pos

  def cb_pan(self, js):
    self.pan_pos = js.current_pos

  def nod(self, repeat = 2):
    first_pos = self.tilt_pos
    for i in range(0,repeat):
      self.bothMore([self.pan_pos, first_pos+0.25])
      time.sleep(0.15)
      self.bothMore([self.pan_pos, first_pos-0.25])
      time.sleep(0.15)


  def baseToObject(self, pos, theta = None):
    if theta is None:
      theta = [self.pan_pos, self.tilt_pos]

    #import pdb
    #pdb.set_trace()
    X = (self.pt_params[1] + pos[0])*cos(theta[0]) + (self.pt_params[2] + pos[1])*sin(theta[0])*sin(theta[1]) - (self.pt_params[3] + pos[2])*sin(theta[0])*cos(theta[1])
    Y =  self.pt_params[0]                         + (self.pt_params[2] + pos[1])              *cos(theta[1]) + (self.pt_params[3] + pos[2])              *sin(theta[1])
    Z = (self.pt_params[1] + pos[0])*sin(theta[0]) - (self.pt_params[2] + pos[1])*cos(theta[0])*sin(theta[1]) + (self.pt_params[3] + pos[2])*cos(theta[0])*cos(theta[1])
    return [X,Y,Z]

  def headFK(self, theta = None): #special case of sensorToBase with pos = [0,0,theta[2]]
    if theta is None:
      theta = [self.pan_pos, self.tilt_pos, 1.0]
    if len(theta) < 3:
      theta = [theta, 1.0]
    [X, Y, Z] = self.baseToObject([0,0,theta[2]],theta)
    return [X,Y,Z]

  #this does not work for negative z!
  def headIK(self, pos):
    t1 = trigSolution([pos[2], pos[0], self.pt_params[1]])
    t2 = trigSolution([sin(t1)*self.pt_params[1]-pos[2], cos(t1)*(pos[1]-self.pt_params[0]), cos(t1)*self.pt_params[2]])

    if self.calc_ik_im_trans:
      Dx = -pos[0] + (self.pt_params[1]*cos(t1) + self.pt_params[2]*sin(t1)*sin(t2))
      Dy =  pos[1] - (self.pt_params[0]         + self.pt_params[2]        *cos(t2))
      Dz =  pos[2] - (self.pt_params[1]*sin(t1) - self.pt_params[2]*cos(t1)*sin(t2))
      #This is the most computationally expensive way to calculate z3 but it avoids checking for div by 0
      #Also you do not even need to calculate this :)
      z3 = sqrt(Dx*Dx+Dy*Dy+Dz*Dz)-self.pt_params[3]
      return [t1,t2,z3]
    else:
      return [t1,t2]

if __name__ == '__main__':
  rospy.init_node('pt_tester')
  pt = PT()
  pt.bothMore([-0.3, 0.6],10,10)
  pt.bothMore([ 0.3, 0.6],10,10)
  pt.bothMore([ 0.3, 0.8],10,10)
  pt.bothMore([-0.3, 0.8],10,10)
  pt.bothMore([ 0.0, 0.7],10,10)
  #rate = rospy.Rate(10)
  #for i in range(0,100):
  #  pt.tilt(0.7)
  #  rate.sleep()

