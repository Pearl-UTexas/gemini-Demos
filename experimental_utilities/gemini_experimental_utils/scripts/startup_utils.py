#!/usr/bin/env python

import rospy
import tf2_ros
import tf
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from std_msgs.msg import *
import geometry_msgs.msg
import numpy as np
from control_msgs.msg import *
from sensor_msgs.msg import *


############## Trakcing through tags
class TagTracking():

	def __init__(self):

		# Adjusitng the Kinect angle
		self.kinect_angle_pub = rospy.Publisher('/tilt_controller/command', Float64)
		self.kinect_angle = Float64()
		self.kinect_angle.data = 0.5
		self.kinect_angle_pub.publish(self.kinect_angle)
		
		# Transform Listener
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		
		self.markerArray = MarkerArray()
		self.legPose = np.zeros(6)
		self.tablePose = np.zeros(6)
		self.gotData = None

		self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
		rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.arPoseMarkerCallback)
		rospy.sleep(1)

	def arPoseMarkerCallback(self, msg):
		if(len(msg.markers)>0):
			self.gotData = True
			mark = msg.markers[0]

			quat = (mark.pose.pose.orientation.x, mark.pose.pose.orientation.x, mark.pose.pose.orientation.z, mark.pose.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)

			self.legPose[0] = mark.pose.pose.position.x
			self.legPose[1] = mark.pose.pose.position.y
			self.legPose[2] = mark.pose.pose.position.z
			self.legPose[3] = euler[0]
			self.legPose[4] = euler[1]
			self.legPose[5] = euler[2]

			## Table Pose
			if(len(msg.markers)>1):
				table = msg.markers[1]
				quat1 = (table.pose.pose.orientation.x, table.pose.pose.orientation.x, table.pose.pose.orientation.z, table.pose.pose.orientation.w)
				euler1 = tf.transformations.euler_from_quaternion(quat1)
				
				self.tablePose[0] = table.pose.pose.position.x
				self.tablePose[1] = table.pose.pose.position.y
				self.tablePose[2] = table.pose.pose.position.z
				self.tablePose[3] = euler1[0]
				self.tablePose[4] = euler1[1]
				self.tablePose[5] = euler1[2]

		else:
			self.gotData = False

	def getObservation(self):
		z = self.legPose - self.tablePose  # Feedback defined by the relative distance between the leg and the table
		return z


def getPOS(tags):
	if not rospy.is_shutdown():
		print "Leg pose = ", tags.legPose
		print "Table Pose =", tags.tablePose
		print "Relative Pose = ", tags.getObservation()


################ Tracking through eef coordinates

class cartesian_eef_tracker():

	def __init__(self, arm_prefix = 'left'):
		self._arm_prefix = arm_prefix

		# Transform Listener
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

	def getObservation(self):
		pos = np.zeros(6)
		trans = self.tfBuffer.lookup_transform(self._arm_prefix + '_ee_link', 'linear_actuator_link', rospy.Time()) #, rospy.Duration(3.0))

		pos[0] = trans.transform.translation.x
		pos[1] = trans.transform.translation.y
		pos[2] = trans.transform.translation.z
		
		quat = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		pos[3] = euler[0]
		pos[4] = euler[1]
		pos[5] = euler[2]
		
		return pos


############# Tracking through joint angles
class joint_angle_values():

	def __init__(self, arm_prefix ='left'):
		self._arm_prefix = arm_prefix
		self.joint_state_old = np.zeros(6)
		self.Subscriber = rospy.Subscriber("/vector/"+ self._arm_prefix + "_arm/joint_states", JointState, self.value_callback)

	def value_callback(self, data):
		joint_names = data.name
		joint_state = data.position

		joint_state = [round(joint_state[i],3) for i in range(len(joint_state))]

		if(not(np.array_equal(joint_state,self.joint_state_old))):
			print "Joint Values = ", joint_state

		self.joint_state_old[:] = joint_state
		# rospy.sleep(1)



################## Main function ###################################
def main():
	###### Arm location using tags #############################################################
	print "Trajectory tracking using tags"
	tags = TagTracking()

	# nPts = input('Enter the number of Points to be recorded : ')

	print "Go to Trajectory Start Point"
	raw_input('Press Enter, if ready ')
	getPOS(tags)

	# for i in range(nPts):
	# 	print "Go to Point #", i
	# 	raw_input('Press Enter, if ready ')
	# 	getPOS(tags)
	# print "Points Recorded"
	# return


	#### Arm location using eef cartesian coordinates and euler angles #############################
	# print "Gathering eef cartesian trajectory"
	# g_pose = cartesian_eef_tracker(arm_prefix = 'left')

	# # nPts = input('Number of Trajecotry point need to be recorded = ')
	# rospy.sleep(1)
	# raw_input('Press Enter when ready')

	# while (not rospy.is_shutdown()):
	# 	print "Recording a point"
	# 	z = g_pose.getObservation()
	# 	print "Recorded Point = ", z
	# 	rospy.sleep(0.5)

	# print "Points Recorded"
	# return


	###### Tracking through Joint angles #######
	# print "Gathering arm joint angle trajectories"
	# sub = joint_angle_values()



if __name__ == '__main__':
	rospy.init_node('arm_poses', anonymous=True)
	main()
	rospy.spin()


