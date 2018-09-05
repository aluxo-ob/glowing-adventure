#! /usr/bin/env python

import roslib
import rospy
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np

from navigans_msgs.msg import ExternalPathAction, ExternalPathGoal, ExternalPathResult, ExternalPathFeedback

# --------------------------------------------------------
class NaviGANsServer:
  def __init__(self):
    # Instantiate action server
    self.server = actionlib.SimpleActionServer('navigans_local_planner', ExternalPathAction, self.execute, False)
    self.server.start()

    # Provide some feedback now, and get ready to publish feedback as the action is executed
    rospy.loginfo('[%s] initialized...' % rospy.get_name() )
    self.feedback = ExternalPathFeedback()

    # Read parameters from configuration file
    self.someValue = rospy.get_param('~useGPU', -1)
    print self.someValue
    # self.someOtherValue = rospy.get_param('~modelFn')
    
  # --------------------------------------------------------
  def execute(self, goal):
    # Main callback function, which is executed when a goal is received

    # publish info to the console for the user
    rospy.loginfo('%s: Executing, NaviGAN local planner' % (rospy.get_name() ) )
    
    # Get fields from the goal description    
    print goal.ignore_obstacles
    #print goal.desired_path.poses[0]

    # number of wayppoints included in Path
    print np.shape(goal.desired_path.poses)[0]

    # Main loop; this should be a while loop, as the robot traverses all the waypoints
    for i in range(0, np.shape(goal.desired_path.poses)[0] ):
      print goal.desired_path.poses[ i ]

    # Publish feedback   
    self.feedback.percent_complete = 0.0101
    
    
    self.server.publish_feedback(self.feedback)

    # Notify when action has ended successfully
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('navigans_control_server')
  server = NaviGANsServer()
  rospy.spin()
