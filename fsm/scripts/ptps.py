import roslib; 
from std_msgs.msg import *
from geometry_msgs.msg import *
import rospy

from pan_tilt import PT
from pose_stuff import PS

from math import pi

def clamp(x, min_val, max_val):
  return max(min(x, max_val), min_val)

class PTPS:
  def __init__(self):
    self.pt = PT()
    self.ps = PS()

    self.searchPattern = [[0.0,0.7], [0.3, 0.6],[-0.3, 0.6],[-0.3, 0.8],[0.3, 0.8]]

    self.pt_rate   = 20
    self.pt_repeat = 20

    self.wait = 50

    self.p1 = 1.0
    self.p2 = 1.0

  def objectSearch(self):
    objectFound = False
    for i in range(0, len(self.searchPattern)):
      if(self.singlePosSearch(self.searchPattern[i])):
        objectFound = True
        break

    return objectFound

  def singlePosSearch(self, inPos):
    self.pt.bothMore(inPos, self.pt_rate, self.pt_repeat)    
    if(self.ps.isTrAllZero()):
      return False
    return True

  def servoToObject(self):
    #try to servo in 10 steps
    for i in range(0,10):
      self.ps._updateTransform()
      print 'objPose: ' + str(self.ps.tr.translation.x) + ' ' + str(self.ps.tr.translation.y)
      tilt_diff = self.ps.tr.translation.y
      pan_diff  = self.ps.tr.translation.x
      if abs(tilt_diff) < 0.03:
        tilt_diff = 0;
      if abs(pan_diff) < 0.03:
        pan_diff = 0;
      if pan_diff + tilt_diff == 0:
        break

      t1 = self.pt.pan_pos  - self.p1*pan_diff
      t2 = self.pt.tilt_pos + self.p2*tilt_diff #opposite direction

      t1 = clamp(t1, -pi/3, pi/3) 
      t2 = clamp(t2, -pi/3, pi/3) 

      print t1,t2

      self.pt.bothMore([t1,t2],5,5)

  def directLookAtObject(self):
    self.ps._updateTransform()
    pos = [self.ps.tr.translation.x, self.ps.tr.translation.y, self.ps.tr.translation.z]
    posInBase = self.pt.baseToObject(pos)
    h_ik = self.pt.headIK(posInBase)
    self.pt.bothMore([h_ik[0], h_ik[1]],10,10)
      
    
if __name__ == '__main__':
  import time
  rospy.init_node('ptps_tester')
  ptps = PTPS()
#  ptps.objectSearch()
#  pos = [-0.55325096249580383, 0.21662694215774536, 1.1187365055084229]
#  posInBase = ptps.pt.baseToObject(pos)
#  h_ik = ptps.pt.headIK(posInBase)
#  print 'IK results: ' + str(h_ik)
#  print 'FK results: ' + str(ptps.pt.headFK(h_ik))
#  print 'Wrt base  : ' + str(posInBase)
  if(ptps.objectSearch()):
    ptps.ps._updateTransform();
    pos = [ptps.ps.tr.translation.x, ptps.ps.tr.translation.y, ptps.ps.tr.translation.z]
    print 'current object pos wrt sensor ' + str(pos)
    posInBase = ptps.pt.baseToObject(pos)
    print 'current object pos wrt base ' + str(posInBase)
    h_ik = ptps.pt.headIK(posInBase)
    print 'IK results: ' + str(h_ik)
    print 'FK results: ' + str(ptps.pt.headFK(h_ik))
#    ptps.servoToObject()
    ptps.directLookAtObject()
    time.sleep(1)

    ptps.ps._updateTransform()
    print ptps.ps.tr
    print ptps.pt.pan_pos
    print ptps.pt.tilt_pos
    pos = [ptps.ps.tr.translation.x, ptps.ps.tr.translation.y, ptps.ps.tr.translation.z]
    posInBase = ptps.pt.baseToObject(pos)
    print posInBase
  else:
    print 'no object found'
