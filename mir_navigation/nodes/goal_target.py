#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import sys
import time
import numpy as np
from geometry_msgs.msg import Point, Twist

class map_navigation():

  def choose(self):

    choice='q'

    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|PRESSE A KEY:")
    rospy.loginfo("|'1': Goal_1: (3.0 , 8.0)")
    rospy.loginfo("|'2': Goal_2: (8.5 , 4.0) ")
    rospy.loginfo("|'3': Goal_3: (-8.0 , 3.0) ")
    rospy.loginfo("|'4': Goal_4: (-0.3 , -5.0) ")
    rospy.loginfo("|'0': Quit ")
    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|WHERE TO GO?")
    choice = input()
    return choice

  def __init__(self):

    # self.path_pub = rospy.Publisher('my_path', Path, latch=True, queue_size=10)
    # self.odom_sub = rospy.Subscriber('my_odom', Odometry, self.odom_cb, queue_size=10)
    # self.path = Path()

    # declare the coordinates of interest
    ###################################################################
    ############ get the position of the goal x and y #################
    ############ run command: rostopic echo /amcl_pose ################
    ##### change the position of the robot and check the x and y ######
    ###################################################################
    self.xGoal_1 = 3.0
    self.yGoal_1 = 8.0

    self.xGoal_2 = 8.5
    self.yGoal_2 = 4.0

    self.xGoal_3 = -8.0
    self.yGoal_3 = 3.0

    self.xGoal_4 = -0.3
    self.yGoal_4 = -5.0
    
    self.goalReached = False

    # initiliaze
    rospy.init_node('map_navigation', anonymous=False)
    choice = self.choose()


    if (choice == 1):

      self.goalReached = self.moveToGoal(self.xGoal_1, self.yGoal_1)

    elif (choice == 2):

      self.goalReached = self.moveToGoal(self.xGoal_2, self.yGoal_2)

    elif (choice == 3):

      self.goalReached = self.moveToGoal(self.xGoal_3, self.yGoal_3)

    elif (choice == 4):

      self.goalReached = self.moveToGoal(self.xGoal_4, self.yGoal_4)

    elif (choice == 0):

      exit(0)

    if (choice!='q'):

        if (self.goalReached):
          rospy.loginfo("Congratulations!")
            #rospy.spin()

        else:
           rospy.loginfo("Hard Luck!")
    
    #if (choice =='q'):
      #exit(0)

    while choice != 'q':
      choice = self.choose()
      if (choice == 1):

        self.goalReached = self.moveToGoal(self.xGoal_1, self.yGoal_1)

      elif (choice == 2):

        self.goalReached = self.moveToGoal(self.xGoal_2, self.yGoal_2)

      elif (choice == 3):

        self.goalReached = self.moveToGoal(self.xGoal_3, self.yGoal_3)

      elif (choice == 4):

        self.goalReached = self.moveToGoal(self.xGoal_4, self.yGoal_4)
      
      elif (choice == 0):

        exit(0)

      if (choice!='q'):

        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          #rospy.spin()

        else:
          rospy.loginfo("Hard Luck!")

  def shutdown(self):
        # stop MIR
        rospy.loginfo("Quit program")
        rospy.sleep()

  # def odom_cb(self, msg):
  #   cur_pose = PoseStamped()
  #   cur_pose.header = msg.header
  #   cur_pose.pose = msg.pose.pose
  #   self.path.header = msg.header
  #   self.path.poses.append(cur_pose)
  #   self.path_pub.publish(self.path)

  def moveToGoal(self,xGoal,yGoal):

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")


      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "/map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/
      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)
      start = rospy.get_time()  #get the current time

      ac.wait_for_result(rospy.Duration(60))

      plan = Point()
      #last_pose = PoseStamped()


      path_length = 0.0
      #plan.begin() = iter(plan)
      #it = plan.begin()
      
      #last_pose = it
      #it += 1

      #while ac.get_state() !=  GoalStatus.SUCCEEDED:
      path_length += np.hypot(plan.position.x, plan.position.y)

      # for a in m:
      #   if it!=plan.end():
      #     path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y )
      #     last_pose = *it
      #     it +=1
      #     return True

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              end = rospy.get_time()  #get the current time
              duration = end - start
              rospy.loginfo("Duration: %s"%duration)
              rospy.loginfo("The global path length: %s"%path_length)
              #rospy.loginfo("Length: %s"%self.path)
              #rospy.Timer(rospy.Duration(GAZE_CONTROLLER_PERIOD), self.handle_gaze_controller)
              #rospy.loginfo("Duration: %s"%rospy.get_name())
              return True

      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False
            

    

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")