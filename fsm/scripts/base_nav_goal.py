import roslib
import rospy
import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import *
from geometry_msgs.msg import *


#typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalMaker(object):
  
  def __init__(self):
    
    
    self.move_base = actionlib.SimpleActionClient("vector_move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    self.move_base.wait_for_server()
      
    rospy.loginfo("Connected to move base server")
    rospy.loginfo("Starting navigation")
    rospy.loginfo("The end")
    
    
  def move(self, pose):
  
    goal = MoveBaseGoal()
    # Use the map frame to define goal poses
    goal.target_pose.header.frame_id = 'map'

    # Set the time stamp to "now"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal there
    goal.target_pose.pose.position.x=pose.position.x
    goal.target_pose.pose.position.y=pose.position.y
    goal.target_pose.pose.position.z=0
    
    goal.target_pose.pose.orientation.x=pose.orientation.x
    goal.target_pose.pose.orientation.y=pose.orientation.y
    goal.target_pose.pose.orientation.z=pose.orientation.z
    goal.target_pose.pose.orientation.w=pose.orientation.w
    
    print
    print "this is the goal. Position : "+str(goal.target_pose.pose.position.x)+", "+str(goal.target_pose.pose.position.y)+", "+str(goal.target_pose.pose.position.z)+" Orientation : "+str(goal.target_pose.pose.orientation.x)+", "+str(goal.target_pose.pose.orientation.y)+", "+str(goal.target_pose.pose.orientation.z)+", "+str(goal.target_pose.pose.orientation.w)
    print 
    
    # Send the goal pose to the MoveBaseAction server
    self.move_base.send_goal(goal)

    # Allow 1 minute to get there
    finished_within_time = self.move_base.wait_for_result(rospy.Duration(120)) 

    # If we don't get there in time, abort the goal
    if not finished_within_time:
      rospy.loginfo("Timed out achieving goal")
      return False
    else:
      state = self.move_base.get_state()
    if state == GoalStatus.SUCCEEDED:
      rospy.loginfo("Goal succeeded!")
      return True
    else:
      return False

  

if __name__ == '__main__':
  rospy.init_node('base_goals')
  
  table = Pose()
  table.position.x = 3.1489
  table.position.y = 1.091
  table.position.z = 0.0

  table.orientation.x = 0.000
  table.orientation.y = 0.000
  table.orientation.z = 0.75681
  table.orientation.w = 0.65364

  other = Pose()
  other.position.x = 0.8636
  other.position.y = 1.2316
  other.position.z = 0.0

  other.orientation.x = 0.000
  other.orientation.y = 0.000
  other.orientation.z = 0.99992
  other.orientation.w = -0.012469

  nav_goal = GoalMaker()

  if(nav_goal.move(table)):
    nav_goal.move(other)

#table: 3.1489; 1.091; 0 | 0; 0; 0.75681; 0.65364
#other: 0.8636; 1.2316; 0 | 0; 0; 0.99992; -0.012469
  

