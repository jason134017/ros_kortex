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

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import math
from numpy.core.defchararray import array, center
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import degrees, e, pi, radians
from std_srvs.srv import Empty
from std_msgs.msg import Int8,String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 #4.5.2
import imutils

from multipledispatch import dispatch
#<build_depend>message_filters</build_depend>
#import message_filters # To Achieve Multiple subscriber
#import keyboard


# Load names of classes
classesFile = "/home/airobot/arm_file/arm_file/classes.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    #init network 
    rospy.loginfo("init network")
    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')
    #wait FSM init
    rospy.Subscriber("/FSM", Int8, self.FSM_callback)
    #rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.getImage)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.getBoxesDistance)
    #rospy.Subscriber('/rgb_to_depth/image_raw', Image, self.getObj)
    #rospy.Subscriber('/rgb/image_raw', Image, self.getObj)

    self.finite_status = 0
    self.obj_x = 0.4
    self.obj_y = -0.4
    self.obj_z = 0.07
    self.flag = True
    self.detect_obj_name = []

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      eef_link = self.arm_group.get_end_effector_link()
      group_names = self.robot.get_group_names()
      
      #self.arm_group.set_goal_position_tolerance(0.001)
      
      
      # Misc variables
      self.box_name = ''
      self.eef_link = eef_link
      self.group_names = group_names

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    #time
    planning_attempts = 100  # planning attempts
    planning_time = 1  # [s] Planning time for computation
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    #
    arm_group.set_num_planning_attempts(planning_attempts)
    arm_group.set_planning_time(planning_time)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success
  def reach_joint_angles_with_list(self,angle, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = angle[0]*pi/180
      joint_positions[1] = angle[1]*pi/180
      joint_positions[2] = angle[2]*pi/180
      joint_positions[3] = angle[3]*pi/180
      joint_positions[4] = angle[4]*pi/180
      joint_positions[5] = angle[5]*pi/180
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 
  def FSM_callback(self,data):
      #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
      self.finite_status = data.data
      
  def get_distance(self,frame):
      img_height = frame.shape[0]
      img_width = frame.shape[1]
      x_center = img_width/2
      y_center = img_height/2
      #rospy.loginfo("The center pixel value is: %s", frame[y_center][x_center])
      distance = float(frame[y_center][x_center])*100.0
      distance_arr = frame[y_center-10:y_center+11,x_center-10:x_center+11]
      distance_arr = distance_arr.flatten()
      median = np.median(distance_arr)
      #rospy.loginfo("The median distance is: %s cm", median*100.0)
      #rospy.loginfo("The distance of center pixel is: %s cm", distance)

      return distance
  def get_distance_from_box(self,frame,box):
    left = box[0]
    top = box[1]
    width = box[2]
    height = box[3]
    img_height = frame.shape[0]
    img_width = frame.shape[1]
    # 640X576   1280X720

    #rospy.loginfo("%sX%s",img_width,img_height)
    x = int(left+width/2)
    y = int(top+height/2)
    #rospy.loginfo("x:%s y:%s",x,y)
    # img_height = frame.shape[0]
    # img_width = frame.shape[1]
    # x_center = img_width/2
    # y_center = img_height/2
    #rospy.loginfo("The center pixel value is: %s", frame[y_center][x_center])
    distance = float(frame[y][x])*100.0
    distance_arr = frame[y-10:y+11,x-10:x+11]
    distance_arr = distance_arr.flatten()
    median = np.median(distance_arr)
    #rospy.loginfo("The median distance is: %s cm", median*100.0)
    #rospy.loginfo("The distance of center pixel is: %s cm", distance)
    crosshair_size=5
    crosshair_color=(255,0,0)
    cv2.rectangle(frame, (x-crosshair_size,y-crosshair_size), (x+crosshair_size,y+crosshair_size), (255,0,0), 1)
    return distance
  def get_degrees_from_box(self,frame,box):
    left = box[0]
    top = box[1]
    width = box[2]
    height = box[3]
    img_height = frame.shape[0]
    img_width = frame.shape[1] 

    x_center = int(left+width/2)
    y_center = int(top+height/2)
    x_l = self.get_distance_from_x_y(frame,x_center-25,y_center)
    x_r = self.get_distance_from_x_y(frame,x_center+25,y_center)

    detha_x = x_l-x_r
    detha_y = 50.0/1280.0*0.9*100
    rospy.loginfo("y:%f x:%f",detha_y,detha_x)

    rad = math.atan2(detha_y,detha_x)
    degree = math.degrees(rad)
    return degree

  def get_distance_from_x_y(self,frame,x,y):
    # left = box[0]
    # top = box[1]
    # width = box[2]
    # height = box[3]
    # img_height = frame.shape[0]
    # img_width = frame.shape[1]
    # # 640X576   1280X720

    # #rospy.loginfo("%sX%s",img_width,img_height)
    # x = int(left+width/2)
    # y = int(top+height/2)
    #rospy.loginfo("x:%s y:%s",x,y)
    # img_height = frame.shape[0]
    # img_width = frame.shape[1]
    # x_center = img_width/2
    # y_center = img_height/2
    #rospy.loginfo("The center pixel value is: %s", frame[y_center][x_center])
    distance = float(frame[y][x])*100.0
    distance_arr = frame[y-10:y+11,x-10:x+11]
    distance_arr = distance_arr.flatten()
    median = np.median(distance_arr)
    #rospy.loginfo("The median distance is: %s cm", median*100.0)
    #rospy.loginfo("The distance of center pixel is: %s cm", distance)
    crosshair_size=5
    crosshair_color=(255,0,0)
    cv2.rectangle(frame, (x-crosshair_size,y-crosshair_size), (x+crosshair_size,y+crosshair_size), (50,0,0), 1)
    return distance
  # def get_distance(self,frame,x,y):
  #     img_height = frame.shape[0]
  #     img_width = frame.shape[1]
  #     # x_center = img_width/2
  #     # y_center = img_height/2
  #     #rospy.loginfo("The center pixel value is: %s", frame[y_center][x_center])
  #     distance = float(frame[y][x])*100.0
  #     distance_arr = frame[y-10:y+11,x-10:x+11]
  #     distance_arr = distance_arr.flatten()
  #     median = np.median(distance_arr)
  #     rospy.loginfo("The median distance is: %s cm", median*100.0)
  #     rospy.loginfo("The distance of center pixel is: %s cm", distance)

  #     return distance
  def draw_crosshair(self,frame, crosshair_size=5, crosshair_color=(255,0,0), stroke=1):
    img_height = frame.shape[0]
    img_width = frame.shape[1]
    x_center = img_width/2
    y_center = img_height/2
    cv2.line(frame, (x_center-crosshair_size,y_center-crosshair_size), (x_center+crosshair_size,y_center+crosshair_size), crosshair_color, stroke)
    cv2.line(frame, (x_center+crosshair_size,y_center-crosshair_size), (x_center-crosshair_size,y_center+crosshair_size), crosshair_color, stroke)
    cv2.rectangle(frame, (x_center-crosshair_size,y_center-crosshair_size), (x_center+crosshair_size,y_center+crosshair_size), (255,0,0), 1)

  def draw_crosshair_from_box(self,frame, crosshair_size=5, crosshair_color=(255,0,0), stroke=1):
    img_height = frame.shape[0]
    img_width = frame.shape[1]
    x_center = img_width/2
    y_center = img_height/2
    cv2.line(frame, (x_center-crosshair_size,y_center-crosshair_size), (x_center+crosshair_size,y_center+crosshair_size), crosshair_color, stroke)
    cv2.line(frame, (x_center+crosshair_size,y_center-crosshair_size), (x_center-crosshair_size,y_center+crosshair_size), crosshair_color, stroke)
    cv2.rectangle(frame, (x_center-crosshair_size,y_center-crosshair_size), (x_center+crosshair_size,y_center+crosshair_size), (255,0,0), 1)

  # def draw_crosshair(self,frame, x, y, crosshair_size=5, crosshair_color=(255,0,0), stroke=1):
  #   img_height = frame.shape[0]
  #   img_width = frame.shape[1]
  #   # x_center = img_width/2
  #   # y_center = img_height/2
  #   cv2.line(frame, (x-crosshair_size,y-crosshair_size), (x+crosshair_size,y+crosshair_size), crosshair_color, stroke)
  #   cv2.line(frame, (x+crosshair_size,y-crosshair_size), (x-crosshair_size,y+crosshair_size), crosshair_color, stroke)
  #   cv2.rectangle(frame, (x-crosshair_size,y-crosshair_size), (x+crosshair_size,y+crosshair_size), (255,0,0), 1)

  def getImage(self,data):
    br = CvBridge()

    #convert ROS image message to OpenCV image 
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="32FC1")

    #resize image
    current_frame = imutils.resize(current_frame, width=600)
    current_frame = cv2.GaussianBlur(current_frame, (5,5) , 0)

    n_bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR)

    distance = self.get_distance(current_frame)

    bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR) 

    self.draw_crosshair(bgr_img, crosshair_size=10, crosshair_color=(0,0,255), stroke=1)

    if(distance != 0.0):
        cv2.putText(bgr_img, str(distance) + " cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA, False)
    else:
        cv2.putText(bgr_img, "Too Close/Far! Out of Range!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA, False)

    cv2.imshow("depth camera", bgr_img)

    cv2.imshow("depth camera (no filter)", n_bgr_img)

    cv2.waitKey(1)
  #some thing same getImage function
  def getBoxesDistance(self,data):
        br = CvBridge()
        #convert ROS image message to OpenCV image 
        current_frame = br.imgmsg_to_cv2(data, desired_encoding="32FC1")

        #org
        org = current_frame
        #org  = imutils.resize(org , width=600)
        org = cv2.GaussianBlur(org, (5,5) , 0)
        #resize image
        current_frame = imutils.resize(current_frame, width=600)
        current_frame = cv2.GaussianBlur(current_frame, (5,5) , 0)

        n_bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR)

        distance = self.get_distance(current_frame)

        bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR) 

        self.draw_crosshair(bgr_img, crosshair_size=10, crosshair_color=(0,0,255), stroke=1)

        if(distance != 0.0):
            cv2.putText(bgr_img, str(distance) + " cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA, False)
        else:
            cv2.putText(bgr_img, "Too Close/Far! Out of Range!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA, False)

        #rospy.loginfo(rospy.has_param('/boxes'))
        # if rospy.has_param('/boxes') :
        #   boxes = rospy.get_param('/boxes')
        #   #indices = rospy.get_param('indices')
        #   rospy.loginfo(len(boxes))
        #   for i in  range(len(boxes)):
        #     #i = i[0]
        #     box = boxes[i]
        #     box_distance = self.get_distance_from_box(org,box)
        #     # left = box[0]
        #     # top = box[1]
        #     # width = box[2]
        #     # height = box[3]
        #     #self.drawPred(frame,classIds[i], confidences[i], left, top, left + width, top + height)
        #     rospy.loginfo("box : %s",box_distance)

        if rospy.has_param('/boxes') and rospy.has_param('/indices'):
            boxes = rospy.get_param('/boxes')
            indices = rospy.get_param('/indices')
            for i in indices:
              #i = i[0]
              box = boxes[i]
              box_distance = self.get_distance_from_box(org,box)
              box_degree = self.get_degrees_from_box(org,box)
              left = box[0]
              top = box[1]
              width = box[2]
              height = box[3]
              #self.drawPred(frame,classIds[i], confidences[i], left, top, left + width, top + height)
              
              rospy.loginfo("box : %s",box_distance)
              if rospy.has_param('/classIds'):
                classIds = rospy.get_param('/classIds')
                #rospy.loginfo("classIds : %s",classIds[i])
                # Get the label for the class name and its confidence
                classId = classIds[i]
                count = 0
                if classes:
                    assert(classId < len(classes))
                    #label = '%s:%s' % (classes[classId], label)
                    rospy.loginfo(classes[classId])
                    try:
                      while True:
                          index = self.detect_obj_name.index(classes[classId]+str(count))
                          count = count+1
                    except:
                      self.detect_obj_name.append(classes[classId]+str(count))
                      img_height = org.shape[0]
                      img_width = org.shape[1]
                      x_center = img_width/2
                      y_center = img_height/2
                      distance_y = ((left+width/2)-x_center)/(1280.0/2.0)*(0.9/2.0)             #in camera base world is lR
                      distance_z = ((top+height/2)-y_center)/(720.0/2.0)*(0.45/2.0)             #hl
                      #rospy.loginfo("center_w:%s center_h:%s",(left+width/2),(top+height/2))
                      #rospy.loginfo("center_x:%s center_y:%s",x_center,y_center)
                      #rospy.loginfo("y:%s z:%s",distance_y,distance_z)
                      #self.add_box_with_name(classes[classId]+str(count),float(box_distance/100),0,0)
                      dz= math.radians(90)-math.radians(box_degree) 
                      if classes[classId] == "drink_carton":
                        self.add_box_with_name(classes[classId]+str(count),float(box_distance/100),-distance_y ,-distance_z,dz=-dz)
                      else:
                        self.add_box_with_name(classes[classId]+str(count),float(box_distance/100),-distance_y ,-distance_z,dz=-dz)
                      #self.add_box_with_name(classes[classId]+str(count),0,0,0)
                      rospy.loginfo("add box:%s %f",classes[classId]+str(count),box_distance)
                      rospy.loginfo("box_degree: %f",box_degree)
                      rospy.loginfo("box_radians diff 90: %f",dz)
                      #text = '%s:%d' % (classes[classId], )
                      #cv2.putText(org, text, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

        else:
          pass
          #rospy.loginfo("not obj detection")
        #cv2.imshow("depth camera", bgr_img)
        #cv2.imshow("depth camera (no filter)", n_bgr_img)
        cv2.imshow("obj", org)
        if rospy.has_param('/boxes'):
          rospy.delete_param('/boxes')
        if rospy.has_param('/indices'):
          rospy.delete_param('/indices')
        if rospy.has_param('/classIds'):
          rospy.delete_param('/classIds')
        self.detect_obj_name = []
        cv2.waitKey(1)
  def getObj(self,data):
    br = CvBridge()

    #convert ROS image message to OpenCV image (BGRA)
    current_frame = br.imgmsg_to_cv2(data)

    bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2BGR)
    if (self.flag):
      self.flag = False
      cv2.imwrite("/home/airobot/test.jpg",bgr_frame)
        # Create a 4D blob from a frame.
    # blob = cv2.dnn.blobFromImage(bgr_frame, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)
    # # Sets the input to the network
    # net.setInput(blob)
    # # Runs the forward pass to get output of the output layers
    # outs = net.forward(getOutputsNames(net))

    # # Remove the bounding boxes with low confidence
    # postprocess(bgr_frame, outs)

    # # Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    # t, _ = net.getPerfProfile()
    # label = 'Inference time: %.2f ms' % (t * 1000.0 / cv2.getTickFrequency())
    # cv2.putText(bgr_frame, label, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

    # cv2.imshow("detection", bgr_frame)

    cv2.waitKey(1)

  #sence
  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.x = self.obj_x
    # box_pose.pose.position.y = self.obj_y
    # box_pose.pose.position.z = self.obj_z # slightly above the end effector

    box_pose.pose.position.x = 0.15
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.2 # slightly above the end effector
    
    self.obj_y = self.obj_y + 0.01

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    
    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  
  #sence
  def add_box_with_name(self,name,x,y,z,dx=0,dy=0,dz=0, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "camera_base"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.orientation.x = dx
    box_pose.pose.orientation.y = dy
    box_pose.pose.orientation.z = dz

    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # slightly above the end effector
    
    #self.obj_y = self.obj_y + 0.01

    #box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.04, 0.14, 0.06))
    
    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    #self.box_name=box_name
    return self.wait_for_state_update_by_name(name,box_is_known=True, timeout=timeout)
  
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      #rospy.loginfo("update box")

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL



  def wait_for_state_update_by_name(self, name,box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      #rospy.loginfo("update box")

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL
  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
   
  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    rospy.loginfo("remove:"+box_name)  
    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  rospy.loginfo(rospy.get_namespace()+"is start")
  rospy.set_param(rospy.get_namespace()+'_moveit', True)
  rate = rospy.Rate(5)
  cnt = 0
  now = 0 
  #example.add_box()
  example.reach_named_position("home")
  #example.robot.arm.pick("box")
  #example.attach_box()
  while not rospy.is_shutdown():
        #rospy.loginfo(example.status)
        if example.status==0:
          #example.add_box()
          #example.attach_box()
          rate.sleep()
          #example.remove_box() 
        if example.status!=0 :
          cnt=cnt+1
          if cnt == 1:
            #example.remove_box()
            #example.attach_box()
            pass        
          #if cnt>2:break
          if success and example.status==1 and now!=example.status:
            now=example.status
            rospy.loginfo(now)
            rospy.loginfo("Reaching Named Target Home...")
            success &= example.reach_named_position("home")
            # rospy.loginfo("Reaching Named Target Vertical...")
            # success &= example.reach_named_position("vertical")
            print (success)
          if success and example.status==2 and now!=example.status:
            now=example.status
            # rospy.loginfo("Reaching Named Target Home...")
            # success &= example.reach_named_position("home")
            rospy.loginfo("Reaching Joint Angles...")  
            success &= example.reach_joint_angles_with_list([0,(340-360),(245-360),0,(275-360),180],tolerance=1) #rad
            print (success)
  #example.detach_box()
  example.remove_box()        
  rospy.on_shutdown()     
  cv2.destroyAllWindows() 
  
  
  
  # if success:
  #   rospy.loginfo("Reaching Named Target Vertical...")
  #   success &= example.reach_named_position("vertical")
  #   print (success)
  
  # if success:
  #   rospy.loginfo("Reaching Joint Angles...")  
  #   success &= example.reach_joint_angles(tolerance=0.01) #rad
  #   print (success)
  
  # if success:
  #   rospy.loginfo("Reaching Named Target Home...")
  #   success &= example.reach_named_position("home")
  #   print (success)

  # if success:
  #   rospy.loginfo("Reaching Cartesian Pose...")
    
  #   actual_pose = example.get_cartesian_pose()
  #   actual_pose.position.z -= 0.2
  #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
  #   print (success)
    
  # if example.degrees_of_freedom == 7 and success:
  #   rospy.loginfo("Reach Cartesian Pose with constraints...")
  #   # Get actual pose
  #   actual_pose = example.get_cartesian_pose()
  #   actual_pose.position.y -= 0.3
    
  #   # Orientation constraint (we want the end effector to stay the same orientation)
  #   constraints = moveit_msgs.msg.Constraints()
  #   orientation_constraint = moveit_msgs.msg.OrientationConstraint()
  #   orientation_constraint.orientation = actual_pose.orientation
  #   constraints.orientation_constraints.append(orientation_constraint)

  #   # Send the goal
  #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  # if example.is_gripper_present and success:
  #   rospy.loginfo("Opening the gripper...")
  #   success &= example.reach_gripper_position(0)
  #   print (success)

  #   rospy.loginfo("Closing the gripper 50%...")
  #   success &= example.reach_gripper_position(0.5)
  #   print (success)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
