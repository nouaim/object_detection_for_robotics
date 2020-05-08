#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String 
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
from tf_object_detection.msg import detection_results

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from random import seed
from random import random

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    

    #rospy.init_node('MoveGroupPythonIntefaceTutorial', anonymous=True)
    
    
    
    moveit_commander.roscpp_initialize(sys.argv)

    #self.subb= rospy.Subscriber("/tf_object_detection_node/result", detection_results, self.go_to_joint_state)
    
    
    

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    #self.__result_sub = rospy.Subscriber("/tf_object_detection_node/result", detection_results, self.go_to_joint_state)

    #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    #rospy.spin()
    
  #def callback4(lll):
  #  rospy.loginfo(lll)
  #  if(lll.names_detected== ['g']):
  #      print("okay")


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    

  

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.  
   # rospy.Subscriber("/tf_object_detection_node/result", detection_results, callback)
  def listener(self):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    
    a=rospy.Subscriber("/tf_object_detection_node/result", detection_results, self.go_to_joint_state)


    rospy.spin()

  



 #if (data.names_detected== ['person']):

  def go_to_joint_state(self, data):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #print("the detected object is", data.names_detected)

    #rospy.loginfo(data)
    
    i=0
    seed(1)
    #print("the detected object is", data.names_detected)
    
    move_group = self.move_group 

    if(data.names_detected== ['write the class of the object you want to detect']):
       
       #ymin,
       #xmin,
       #ymax,
       #xmax,    
       print("the detected object is", data.names_detected)
       joint_goal = move_group.get_current_joint_values()
       #joint_goal[0] = 0.42 - 0.25
       #joint_goal[1] = 0.33 +0.4
       joint_goal[2] = 0
       #joint_goal[0]= (data.rectangles[2] + data.rectangles[0])/2  - 0.25
       #joint_goal[1]= (data.rectangles[3] + data.rectangles[1])/2 
       #joint_goal[0] = 0
       joint_goal[1] = 0.3
       #joint_goal[2] = 0


       #joint_goal[3] = 0.17
       #joint_goal[4] = 0.3490659 
       # The go command can be called with joint values, poses, or without any
       # parameters if you have already set the pose or joint target for the group
       move_group.go(joint_goal, wait=True)
       # Calling ``stop()`` ensures that there is no residual movement
       #move_group.stop()
       ## END_SUB_TUTORIAL
       # For testing:
       current_joints = move_group.get_current_joint_values()     
       rospy.sleep(20)
        

    return all_close(joint_goal, current_joints, 0.01)      
        
        
       
      

"""

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    

    #pose_goal.position.x = 240
    #pose_goal.position.y = -130
    #pose_goal.position.z = 60

    pose_goal.position.x = 1.5
    pose_goal.position.y = 1.5
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

"""    
    

def main(args):
  

  try:
    print ("----------------------------------------------------------")
    print ("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    raw_input()
    #obj = MoveGroupPythonIntefaceTutorial()
    #obj.listener()
    print ("Wait for the class of the object")
    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    #raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    tutorial.listener()
    #tutorial.listener2()
    
    #tutorial.listener()
    #tutorial.go_to_joint_state()
    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
     
if __name__ == '__main__':
    main(sys.argv)
    

   #print "============ Press `Enter` to execute a movement using a pose goal ..."
    #raw_input()
    #tutorial.go_to_pose_goal()
