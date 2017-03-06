#!/bin/env python
import roslib; 
from geometry_msgs.msg import *
import rospy
import tf 
import time

class PS:
  def __init__(self, use_tf = False):
    if not use_tf:
      self.objPoseSub = rospy.Subscriber("/baris/objectTransform", Transform, self.cb)
    else:
      self.listener = tf.TransformListener()
      
    self._use_tf = use_tf

    self._tr = Transform()

  def cb(self, inTransform):
    self._tr = inTransform
    #print   self._tr

  def isTrAllZero(self):
    tmp = self._tr.translation.x + self._tr.translation.y + self._tr.translation.z + \
          self._tr.rotation.x + self._tr.rotation.y + self._tr.rotation.z + self._tr.rotation.w

    return tmp == 0
    
  def fillTransform(self,trans_tuple, rot_tuple):
    self._tr.translation.x = trans_tuple[0]
    self._tr.translation.y = trans_tuple[1]
    self._tr.translation.z = trans_tuple[2]
    
    self._tr.rotation.x = rot_tuple[0]
    self._tr.rotation.y = rot_tuple[1]
    self._tr.rotation.z = rot_tuple[2]
    self._tr.rotation.w = rot_tuple[3]
    
  def getTransform(self):
    if self._use_tf:
      self._updateTransform()
    
    return self._tr
    
  def getTransformWrtBase(self):
    if not use_tf:
      raise Exception('Not using tf. Initialize this class with use_tf = True in order to use this function')
    try:
      (self._tr.translation, self._tr.rotation) = self.listener.lookupTransform('/kinect_rgb_optical_frame', '/main_object', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Cannot lookupTransform'
    
  def _updateTransform(self):
    if not self._use_tf:
      return False
    else:
      try:
        (trans, rot) = self.listener.lookupTransform('/kinect_rgb_optical_frame', '/main_object', rospy.Time(0))
        self.fillTransform(trans, rot)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return False
    return True
    
    

if __name__ == '__main__':
  rospy.init_node('ps_tester')
  ps = PS(True)
  #rospy.spin()
  while not rospy.is_shutdown():
    print ps.getTransform()
    time.sleep(0.1)
