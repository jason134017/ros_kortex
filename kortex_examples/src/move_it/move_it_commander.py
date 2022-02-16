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
import cv_bridge
import rospy
import numpy as np
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import copy
from math import degrees, e, pi, radians
from std_srvs.srv import Empty
from std_msgs.msg import Int8,String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
import cv2 #4.5.2
import imutils
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from multipledispatch import dispatch
#if need Writing Multiple Subscribers to get rgb and depth image
#<build_depend>message_filters</build_depend>
import message_filters # To Achieve Multiple subscriber
#import keyboard
#from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from kortex_driver.msg import BaseCyclic_Feedback
from pynput import keyboard
# Load names of classes
classesFile = "/home/airobot/arm_file/arm_file/classes.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

camera_name ="kinect"
camera_name ="realsense"


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True



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
  
    #rospy.Subscriber('/imu', Imu, self.InitImu)
    #rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.arm_feedback)
    
    #rospy.Subscriber('/rgb/image_raw', Image, self.getObj)
    #rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.getBoxesDistance)
    rospy.Subscriber('/dope/detected_objects', Detection3DArray, self.nv_detected_objects)
    
    if(camera_name=="kinect"):
        rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.getImage)
        # rospy.Subscriber('/rgb_to_depth/image_raw', Image, self.getObj)
        #rospy.Subscriber('/rgb_to_depth/image_raw', Image, self.InitRGBD)
        rospy.Subscriber('/rgb/image_raw', Image, self.InitRGBD)
    else:
       rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.getImage)
       rospy.Subscriber('/camera/color/image_raw', Image, self.InitRGBD_Realsense)


    #
    self.tf_listener = tf.TransformListener()
    


    # Writing Multiple Subscribers to get rgb and depth image
    # self.rgb_sub = message_filters.Subscriber('/rgb_to_depth/image_raw', Image)
    # # Node is subscribing to the /depth_to_rgb/image_raw topic
    # self.depth_sub = message_filters.Subscriber('/depth_to_rgb/image_raw', Image)
    self.pub2 = rospy.Publisher('crosshair', Image, queue_size=10)
    self.finite_status = 0
    self.test_status = 0
    self.obj_x = 0.4
    self.obj_y = -0.4
    self.obj_z = 0.07
    self.flag = True
   
    # use detectron2
    self.detect_obj_name = []
    # use nv dope 
    self.nv_Detection3DArray = None
    self.is_tf_camera_init = False
    self.tf_camera = 0.0
    self.DepthToRGB_camera = None
    self.crosshairDistance = 0.0
    self.image_hidden = False
    self.tmpBox_pose = geometry_msgs.msg.PoseStamped()
    self.detect_count =0
    # Create a publisher for displaying gripper poses
    self.gripper_pose_pub = rospy.Publisher('gripper_pose', geometry_msgs.msg.PoseStamped)
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
      # init arm_group
      planning_attempts = 1000  # planning attempts 100
      planning_time = 0.5  # [s] Planning time for computation
      self.arm_group.set_num_planning_attempts(planning_attempts)
      self.arm_group.set_planning_time(planning_time)
      self.arm_group.set_max_acceleration_scaling_factor(1)
      self.arm_group.set_max_velocity_scaling_factor(1)
      #print(self.arm_group.max_velocity_scaling_factor())
      
      # Misc variables
      self.box_name = ''
      self.eef_link = eef_link
      rospy.loginfo(eef_link)
      self.group_names = group_names

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
      # Remove leftover objects from a previous run
      self.scene.remove_world_object("test")


      #  # Give each of the scene objects a unique name        
      # table_id = 'table'
      # box1_id = 'box1'
      # box2_id = 'box2'
      # target_id = 'target'
      # REFERENCE_FRAME ="base_link"
      # # Remove leftover objects from a previous run
      # self.scene.remove_world_object(table_id)
      # self.scene.remove_world_object(box1_id)
      # self.scene.remove_world_object(box2_id)
      # self.scene.remove_world_object(target_id)

      # # Set the height of the table off the ground
      # table_ground = 0.75

      # # Set the dimensions of the scene objects [l, w, h]
      # table_size = [0.2, 0.7, 0.01]
      # box1_size = [0.1, 0.05, 0.05]
      # box2_size = [0.05, 0.05, 0.15]

      # # Set the target size [l, w, h]
      # target_size = [0.02, 0.01, 0.12]

      # # Add a table top and two boxes to the scene
      # table_pose = geometry_msgs.msg.PoseStamped()
      # table_pose.header.frame_id = REFERENCE_FRAME
      # table_pose.pose.position.x = 0.25
      # table_pose.pose.position.y = 0.0
      # table_pose.pose.position.z = table_ground + table_size[2] / 2.0
      # table_pose.pose.orientation.w = 1.0
      # self.scene.add_box(table_id, table_pose, table_size)

      # box1_pose =geometry_msgs.msg.PoseStamped()
      # box1_pose.header.frame_id = REFERENCE_FRAME
      # box1_pose.pose.position.x = 0.21
      # box1_pose.pose.position.y = -0.1
      # box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
      # box1_pose.pose.orientation.w = 1.0   
      # self.scene.add_box(box1_id, box1_pose, box1_size)

      # box2_pose =geometry_msgs.msg.PoseStamped()
      # box2_pose.header.frame_id = REFERENCE_FRAME
      # box2_pose.pose.position.x = 0.19
      # box2_pose.pose.position.y = 0.13
      # box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
      # box2_pose.pose.orientation.w = 1.0   
      # self.scene.add_box(box2_id, box2_pose, box2_size)       

      # # Set the target pose in between the boxes and on the table
      # target_pose = geometry_msgs.msg.PoseStamped()
      # target_pose.header.frame_id = REFERENCE_FRAME
      # target_pose.pose.position.x = 0.22
      # target_pose.pose.position.y = 0.0
      # target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
      # target_pose.pose.orientation.w = 1.0

      # # Add the target object to the scene
      # self.scene.add_box(target_id, target_pose, target_size)
      # # Initialize the grasp pose to the target pose
      # grasp_pose = target_pose

      # # Shift the grasp pose by half the width of the target to center it
      # grasp_pose.pose.position.y -= target_size[1] / 2.0

      # Generate a list of grasps
      # grasps = self.make_grasps(grasp_pose, [target_id])

      # # Publish the grasp poses so they can be viewed in RViz
      # for grasp in grasps:
      #     self.gripper_pose_pub.publish(grasp.grasp_pose)
      #     rospy.sleep(0.2)
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True
    


  def reach_named_position(self, target):
    arm_group = self.arm_group
    #time
    planning_attempts = 100  # planning attempts 100
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
  def reach_joint_angles_with_list(self,angle, tolerance=0.01):
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
      joint_positions[0] = angle[0]
      joint_positions[1] = angle[1]
      joint_positions[2] = angle[2]
      joint_positions[3] = angle[3]
      joint_positions[4] = angle[4]
      joint_positions[5] = angle[5]
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

  def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.arm_group
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.4

        #pose_goal.orientation.w = 0.0246440951065
        pose_goal.position.x = 0.134106966002
        pose_goal.position.y = 0.00135022537484
        pose_goal.position.z = 0.198529055253

     

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.arm_group.go(wait=True)
        if not plan.joint_trajectory.points:
           print("Error")
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.arm_group.get_current_pose().pose
        print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal(self,w=1.0,x=0.4,y=0.1,z=0.4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.arm_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        print(self.arm_group.get_current_pose().pose)
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
        current_pose = self.arm_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

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
  ##############################
  # calculate distance
  ##############################   
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
  
  ##############################
  # callback
  ############################## 
  def getImage(self,data):
    br = CvBridge()

    #convert ROS image message to OpenCV image 
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="32FC1")
    
    #resize image
    current_frame = imutils.resize(current_frame, width=600)
    current_frame = cv2.GaussianBlur(current_frame, (5,5) , 0)
    
    n_bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR)

    distance = self.get_distance(current_frame)
    if (camera_name=="realsense"):
      distance/=1000
    self.crosshairDistance = distance
    # if (not self.is_tf_camera_init):
    #   self.InitCameraWorldSpace(distance)
    #   self.is_tf_camera_init = True
    #   rospy.loginfo("Initializing camera transform")


    bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR) 

    self.draw_crosshair(bgr_img, crosshair_size=10, crosshair_color=(0,0,255), stroke=1)
    #self.draw_crosshair(current_frame, crosshair_size=10, crosshair_color=(255,255,255), stroke=1)
    if(distance != 0.0):
        cv2.putText(bgr_img, str(distance) + " cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA, False)
    else:
        cv2.putText(bgr_img, "Too Close/Far! Out of Range!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA, False)
  
    # cv2.imshow("depth camera", bgr_img)

    #cv2.imshow("depth camera (no filter)", n_bgr_img)
    #save init

    # key = cv2.waitKey(1)
    # if key == ord('s'): # Esc
    #   self.InitCameraWorldSpace(distance)
    #   self.is_tf_camera_init = True
    #   rospy.loginfo("Initializing camera transform")

  def InitCameraWorldSpace(self,distance):
    self.tf_camera = distance/100
    #if (camera_name=="realsense"):
   
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
        cv2.imshow("depth camera", bgr_img)
        #cv2.imshow("depth camera (no filter)", n_bgr_img)
        #cv2.imshow("obj", org)
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
    # if (self.flag):
    #   self.flag = False
    #   cv2.imwrite("/home/airobot/test.jpg",bgr_frame)
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

    #cv2.imshow("detection", bgr_frame)

    #cv2.waitKey(1)
  def InitRGBD(self,data):
    br = CvBridge()

    #convert ROS image message to OpenCV image (BGRA)
    current_frame = br.imgmsg_to_cv2(data)
    bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2BGR)
    self.draw_crosshair(bgr_frame, crosshair_size=10, crosshair_color=(0,0,255), stroke=1)

    if(self.crosshairDistance != 0.0):
        cv2.putText(bgr_frame, str(self.crosshairDistance) + " cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA, False)
    else:
        cv2.putText(bgr_frame, "Too Close/Far! Out of Range!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA, False)
    if (not self.image_hidden):
      cv2.imshow("camera", bgr_frame)
    #save init
    key = cv2.waitKey(1)
    if key == ord('s'): # save
      self.InitCameraWorldSpace(self.crosshairDistance)
      time.sleep(1)
      self.is_tf_camera_init = True
      rospy.loginfo("Initializing camera transform")
    if key == ord('h'): # Esc
      self.image_hidden = not self.image_hidden
      cv2.destroyWindow("camera")
    if key == ord('q'): # Esc
      self.test_status = 888
    if key == ord('n'): # Esc
      self.test_status = 777

  def InitRGBD_Realsense(self,data):
    br = CvBridge()

    #convert ROS image message to OpenCV image (BGRA)
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="passthrough")
    #bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2BGR)
    bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
    self.draw_crosshair(bgr_frame, crosshair_size=10, crosshair_color=(0,0,255), stroke=1)

    if(self.crosshairDistance != 0.0):
        cv2.putText(bgr_frame, str(self.crosshairDistance) + " cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA, False)
    else:
        cv2.putText(bgr_frame, "Too Close/Far! Out of Range!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA, False)
    if (not self.image_hidden):
      cv2.imshow("camera", bgr_frame)
    #save init
    key = cv2.waitKey(1)
    if key == ord('s'): # save
      self.InitCameraWorldSpace(self.crosshairDistance)
      self.is_tf_camera_init = True
      time.sleep(2)
      rospy.loginfo("Initializing camera transform")
    if key == ord('h'): # Esc
      self.image_hidden = not self.image_hidden
      cv2.destroyWindow("camera")
    if key == ord('q'): # Esc
      self.test_status = 888
    if key == ord('n'): # Esc
      self.test_status = 777
  def InitImu(self,data):
    rospy.loginfo(data)
    pass
  def nv_detected_objects(self,data):
    self.nv_Detection3DArray = data
    if (self.is_tf_camera_init):
      self.add_nvbox()
    else:
      self.detect_count +=1
      print("wait tf camera init")
    if self.detect_count >=100:
      self.detect_count =0
      rospy.loginfo("wait tf camera init")
  #sence
  def add_nvbox(self):
    #print(self.nv_Detection3DArray.detections)
    for i in self.nv_Detection3DArray.detections:
        # print(i.bbox)
        # center: 
        #   position: 
        #     x: -0.0910059173315
        #     y: 0.114222536905
        #     z: 0.456539992161
        #   orientation: 
        #     x: 0.292211169064
        #     y: 0.410156978635
        #     z: 0.541593039891
        #     w: 0.673097960696
        # size: 
        #   x: 0.0835550022125
        #   y: 0.0711210012436
        #   z: 0.0660559988022  
          self.add_box_with_name("test",x=i.bbox.center.position.x,y=i.bbox.center.position.y,z=i.bbox.center.position.z,
          dx=i.bbox.center.orientation.x,dy=i.bbox.center.orientation.y,dz=i.bbox.center.orientation.z,dw=i.bbox.center.orientation.w,
          size_x=i.bbox.size.x,size_y=i.bbox.size.y,size_z=i.bbox.size.z)

  def arm_feedback(self,data):
    print(data)
  ##############################
  # moveit plan_sence
  ############################## 
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
  def add_box_with_name(self,name,x,y,z,dx=0,dy=0,dz=0,dw=1.0,size_x=1.0,size_y=1.0,size_z=1.0, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = name
    self.box_name = name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = PoseStamped()
    # box_pose.header.frame_id = "camera_base"
   
    # orientation_list = [dx, dy, dz, dw]
    # (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # # quaternion = quaternion_from_euler (pitch,yaw,roll)
    # # quaternion = quaternion_from_euler (-yaw,-roll,-pitch)
    # # quaternion = quaternion_from_euler (0,0,0)
    # # quaternion = quaternion_from_euler (-yaw,-roll,-pitch)
    

    # box_pose.pose.position.x = z 
    # box_pose.pose.position.y = -x
    # box_pose.pose.position.z = -y
    # # print(x,y,z)
    # box_pose.pose.orientation.w = dw
    # box_pose.pose.orientation.x = dz
    # box_pose.pose.orientation.y = -dx
    # box_pose.pose.orientation.z = -dy


    # depth
    #box_pose.header.frame_id = "depth_camera_link"
    if(camera_name=="kinect"):
      box_pose.header.frame_id = "rgb_camera_link"
    elif(camera_name=="realsense"):
      # box_pose.header.frame_id = "camera_color_frame"
      box_pose.header.frame_id = "camera_color_optical_frame"
   
    orientation_list = [dx, dy, dz, dw]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # quaternion = quaternion_from_euler (pitch,yaw,roll)
    # quaternion = quaternion_from_euler (-yaw,-roll,-pitch)
    # quaternion = quaternion_from_euler (0,0,0)
    # quaternion = quaternion_from_euler (-yaw,-roll,-pitch)
    
    #[0.032082906679588165, 0.002493106534485829, -0.0037813835614916233]
    box_pose.pose.position.x = x#-0.04#-0.03
    box_pose.pose.position.y = y#+0.037
    box_pose.pose.position.z = z#+0.032
    #print(x,y,z)
    box_pose.pose.orientation.w = dw
    box_pose.pose.orientation.x = dx
    box_pose.pose.orientation.y = dy
    box_pose.pose.orientation.z = dz

    # # depth
    # #box_pose.header.frame_id = "depth_camera_link"
    # box_pose.header.frame_id = "rgb_camera_link"
   
    # orientation_list = [dx, dy, dz, dw]
    # (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # print(roll)
    # quaternion = quaternion_from_euler (roll, pitch+pi/2, yaw)
    # # quaternion = quaternion_from_euler (-yaw,-roll,-pitch)
    # # quaternion = quaternion_from_euler (0,0,0)
    # # quaternion = quaternion_from_euler (-yaw,-roll,-pitch)
    
    # #[0.032082906679588165, 0.002493106534485829, -0.0037813835614916233]
    # box_pose.pose.position.x = x-0.04#-0.03
    # box_pose.pose.position.y = y#+0.037
    # box_pose.pose.position.z = z#+0.032
    # #print(x,y,z)
    # # box_pose.pose.orientation.w = dw
    # # box_pose.pose.orientation.x = dx
    # # box_pose.pose.orientation.y = dy
    # # box_pose.pose.orientation.z = dz

    # box_pose.pose.orientation.w = quaternion[3]
    # box_pose.pose.orientation.x = quaternion[0]
    # box_pose.pose.orientation.y = quaternion[1]
    # box_pose.pose.orientation.z = quaternion[2]



    # (trans,rot) = self.tf_listener.lookupTransform("base_link", "rgb_camera_link", rospy.Time(0))
    #print(trans,rot)
    #print(trans)

    # copy
    self.tmpBox_pose = box_pose


    #use  quaternion
    quaternion = quaternion_from_euler (pitch,yaw,roll)
    # print(math.degrees(roll),math.degrees(pitch) ,math.degrees(yaw) )
    # box_pose.pose.orientation.x = quaternion[0]
    # box_pose.pose.orientation.y = quaternion[1]
    # box_pose.pose.orientation.z = quaternion[2]
    # box_pose.pose.orientation.w = quaternion[3]

    # box_pose.pose.position.x = -x 
    # box_pose.pose.position.y = -y 
    # box_pose.pose.position.z = -z   # slightly above the end effector

    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.orientation.x = 0
    # box_pose.pose.orientation.y = 0
    # box_pose.pose.orientation.z = 0

    # box_pose.pose.position.x = 0
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0   # slightly above the end effector
    # print(dw,dx,dy,dz)
    # print(x,y,z)

    #self.obj_y = self.obj_y + 0.01

    #box_name = "box"
    # size_tf = (size_z, size_x, size_y)
    #size_tf = (size_x*0.8, size_y*0.8, size_z*0.8)
    size_tf =  (size_x, size_y, size_z)
    # size_tf = (0.2,0.1, 0.1)
    scene.add_box(box_name, box_pose, size=size_tf)
    #scene.add_cylinder(box_name, box_pose,height=0.12,radius=0.01)
    #scene.addCylinder()
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

  # To Achieve Multiple subscriber   
  def MultipleSubscriber(self, ros_rgb, ros_depth):
      br = CvBridge()
      #convert ROS image message to OpenCV image 
      depth_frame = br.imgmsg_to_cv2(ros_depth, desired_encoding="32FC1")
      rgb_frame = br.imgmsg_to_cv2(ros_rgb, "bgr8")

      #resize image
      depth_frame = imutils.resize(depth_frame, width=600)
      rgb_frame = imutils.resize(rgb_frame, width=600)
      
      depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_GRAY2BGR)
      rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGRA2BGR)
      test = depth_frame.copy()
      #If we want to stablize, don't use imshow and waitKey(1). Use a publisher to send the data out and use rqt_gui to see the frame.
      # cv2.imshow("depth camera", depth_frame)
      # cv2.imshow("rgb camera", rgb_frame)
      # rospy.loginfo("run")
      #cv2.imshow("rgb camera", test)
      #cv2.waitKey(1)

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def main():
  example = ExampleMoveItTrajectories()
  tf_listener = tf.TransformListener()
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
  #example.reach_named_position("home")
  #example.robot.arm.pick("box")
  #example.attach_box()
  # listener = keyboard.Listener(
  # on_press=on_press,
  # on_release=on_release)
  # listener.start()


  # ts = message_filters.ApproximateTimeSynchronizer([example.rgb_sub, example.depth_sub], 10, 0.1)
  # ts.registerCallback(example.MultipleSubscriber)


  # with keyboard.Events() as events:
  #   for event in events:
  #       if event.key == keyboard.Key.esc:
  #           break
  #       else:
  #           print('Received event {}'.format(event))
  success =True
  
  rospy.loginfo("Reaching Named Target retract...")
  #success &= example.reach_named_position("retract")
  success &= example.reach_named_position("retract")
  rospy.loginfo("Opening the gripper...")
  success &= example.reach_gripper_position(0)
  success &= example.reach_joint_angles_with_list(angle=[0,0,-0.5*pi,0,-0.5*pi,pi],tolerance=0.01)

  while not rospy.is_shutdown():
        rospy.loginfo(example.test_status)
        # init tf
        # (trans,rot) = tf_listener.lookupTransform("base_link", "shoulder_link", rospy.Time(0))
        # print(trans,rot)
        br = tf.TransformBroadcaster() 
        if (example.is_tf_camera_init):
          # br.sendTransform((example.tf_camera, 0.0, 0.17),
          #                 quaternion_from_euler(0, 0, pi),
          #                 rospy.Time.now(),
          #                 "camera_base",
          #                 "base_link")

          #shoulder_link->base_link
          if(camera_name == "kinect"):
            br.sendTransform((example.tf_camera, 0.0, 0.15643),
                            quaternion_from_euler(0, 0, pi),
                            rospy.Time.now(),
                            "camera_base",
                            "base_link")
          elif(camera_name=="realsense"):
            br.sendTransform((example.tf_camera, 0.0, 0.15643),
                quaternion_from_euler(0, 0, pi),
                rospy.Time.now(),
                "camera_link",
                "base_link")
        if (example.test_status==0):
          if (example.is_tf_camera_init):
            example.test_status=1
          
          #example.add_box()
          #example.attach_box()
          #rospy.loginfo(example.nv_Detection3DArray)
          rate.sleep()
          #example.remove_box()
        if (example.test_status==1):
          # if success:
          #   rospy.loginfo("Reaching Cartesian Pose init...")
            
          #   actual_pose = example.get_cartesian_pose()
            
          #   actual_pose.position.x +=  0.2
          #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
          #   #print("go to /n",example.tmpBox_pose)

          #   print (success)
            example.test_status=2 
            #example.arm_group.pickup("Box_0")
            pass
        if (example.test_status==2):
          
          rospy.loginfo("Reaching Cartesian Pose...")
          
          actual_pose = example.get_cartesian_pose()
          
          # try:
          #   (trans,rot) = tf_listener.lookupTransform("camera_base", "base_link", rospy.Time(0))
          #   #print(trans,rot)
          # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          #     print("get tf false")
          # print(type(actual_pose))
          mpose_transf = tf_listener.transformPose("base_link",example.tmpBox_pose)
          print(mpose_transf)


          actual_pose.position.z = mpose_transf.pose.position.z +0.15
          actual_pose.position.x = mpose_transf.pose.position.x
          actual_pose.position.y =  mpose_transf.pose.position.y
          # if(actual_pose.position.x>0):
          #   actual_pose.position.x-=0.015
          # if(actual_pose.position.y<0):
          #   actual_pose.position.y-=0.03
          # if(actual_pose.position.x>0):
          #   actual_pose.position.x+=0.05
          success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
          #print("go to /n",example.tmpBox_pose)
          print (success)


          rospy.loginfo("Reaching Joint Angles...")  
          orientation_list = [mpose_transf.pose.orientation.x, mpose_transf.pose.orientation.y, mpose_transf.pose.orientation.z, mpose_transf.pose.orientation.w]
          (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

          # success &= example.reach_joint_angles_with_list(angle=[0,0,0,0,0, yaw],tolerance=0.01)
          
          if(success):
            example.test_status=2
          # example.go_to_pose_goal(w=1.0,x=example.tmpBox_pose.pose.position.x,
          # y=example.tmpBox_pose.pose.position.y,z=example.tmpBox_pose.pose.position.z+0.15)

        if (example.test_status==3):
          rospy.loginfo("Reaching Cartesian Pose...")
          actual_pose = example.get_cartesian_pose()
          actual_pose.position.z -= 0.02
          success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
          if(not success):
            print("not down")
            example.test_status=4
        if(example.test_status ==4):
          #example.attach_box()
          #example.arm_group.tool_frame.pick("test")
          example.arm_group.pick("test")
          #example.robot.tool_frame.pick("test")
          rospy.loginfo("Closing the gripper 50%...")
          success &= example.reach_gripper_position(0.5)
          # rospy.loginfo("Opening the gripper...")
          # success &= example.reach_gripper_position(0)
          # print (success)
          example.test_status=5
        if(example.test_status ==5):
          #example.detach_box()
          example.test_status=6
        if (example.test_status==777):
          example.test_status=2
        if (example.test_status==888):
          #example.remove_box()
          #example.reach_named_position("home")
          #cartesian_plan, fraction = example.plan_cartesian_path()
          # example.go_to_pose_goal(w=1.0,x=example.tmpBox_pose.pose.position.x,
          # y=example.tmpBox_pose.pose.position.y,z=example.tmpBox_pose.pose.position.z+0.15)
          #example.test_status=3
          break
        # if example.test_status!=0 :
        #   cnt=cnt+1
        #   if cnt == 1:
        #     #example.remove_box()
        #     #example.attach_box()
        #     pass        
        #   #if cnt>2:break
        #   if success and example.test_status==1 and now!=example.test_status:
        #     now=example.status
        #     rospy.loginfo(now)
        #     rospy.loginfo("Reaching Named Target Home...")
        #     success &= example.reach_named_position("home")
        #     # rospy.loginfo("Reaching Named Target Vertical...")
        #     # success &= example.reach_named_position("vertical")
        #     print (success)
        #   if success and example.test_status==2 and now!=example.test_status:
        #     now=example.test_status
        #     # rospy.loginfo("Reaching Named Target Home...")
        #     # success &= example.reach_named_position("home")
        #     rospy.loginfo("Reaching Joint Angles...")  
        #     success &= example.reach_joint_angles_with_list([0,(340-360),(245-360),0,(275-360),180],tolerance=1) #rad
        #     print (success)
  #example.detach_box()
  #example.remove_box()
  # listener.stop()        
      
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
  exit(main())
