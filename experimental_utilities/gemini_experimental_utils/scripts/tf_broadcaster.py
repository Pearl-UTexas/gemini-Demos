#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from gemini_experimental_utils.srv import object_frame_data

class broacast_frame():
    
    def __init__(self, parent, child):
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()
        # self.t.header.frame_id = "linear_actuator_link"
        self.t.header.frame_id = parent
        self.t.child_frame_id = child  
        self.rate = rospy.Rate(20.0) 
        self.dataInit = False     

    def broadcast(self):
        self.t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.t)

    def service_client(self):
        rospy.wait_for_service('object_frame_tf')
        handle = rospy.ServiceProxy('object_frame_tf', object_frame_data)
        msg = handle()
        self.t.transform.translation.x = msg.pose.position.x 
        self.t.transform.translation.y = msg.pose.position.y 
        self.t.transform.translation.z = msg.pose.position.z
        self.t.transform.rotation.x = msg.pose.orientation.x
        self.t.transform.rotation.y = msg.pose.orientation.y
        self.t.transform.rotation.z = msg.pose.orientation.z
        self.t.transform.rotation.w = msg.pose.orientation.w
        return

if __name__ == '__main__':  
    rospy.init_node('tf_broadcaster', anonymous=True)
    parent = rospy.get_param('~parent_frame')
    child = rospy.get_param('~child_frame')
    tf_br = broacast_frame(parent, child)
    # tf_br.listener()

    while((not rospy.is_shutdown())):
        tf_br.service_client()
        tf_br.broadcast()

    rospy.spin()
        
