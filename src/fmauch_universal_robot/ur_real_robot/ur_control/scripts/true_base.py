#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import threading

from torch import int32

import tf
import sys
import cv2
import time
import rospy
import random
import pprint
import image_geometry
import message_filters
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
import copy
import tf2_ros
import geometry_msgs.msg
import traceback
import math
import select, termios, tty
import moveit_commander
from sensor_msgs.msg import JointState


# from PIL import Image,ImageDraw
# import numpy as np 
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
 
class TrueBase(object):
    def __init__(self, group_):
        self.tf_listener = tf.TransformListener()
        self.action_params = ['rgb_img', 'depth_img', 'camera_model', 'timestamp']
        self.group = group_
        self.effector = sys.argv[1] if len(sys.argv) > 1 else 'tool0'
        self.clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        self.br = tf2_ros.TransformBroadcaster()
        # 发布螺栓姿态发布
        self.bolt_pos_pub = rospy.Publisher('/NSPlanner/bolt_pose', geometry_msgs.msg.PoseStamped, queue_size =10 )
        self.x_shift= 0
        self.y_shift= 0
        self.z_shift= 0          

    def set_arm_pose(self, group, pose, effector):

        '''
        输入slef.group以及目标位置位姿，使机器运动到该状态
        '''

        # 接受关节状态信息
        joint_states = rospy.wait_for_message("joint_states",JointState)
        joint_pose = joint_states.position
        if (joint_pose[5] > math.pi):
            joints = {}
            joints["elbow_joint"] = joint_pose[0]
            joints["shoulder_lift_joint"] = joint_pose[1]
            joints["shoulder_pan_joint"] = joint_pose[2]
            joints["wrist_1_joint"] = joint_pose[3]
            joints["wrist_2_joint"] = joint_pose[4]
            joints["wrist_3_joint"] = joint_pose[5]-2*math.pi
            # 基于moveit运动
            group.set_joint_value_target(joints)
            plan_success, plan, planning_time, error_code = group.plan()
            # 规划成功则执行，失败跳出
            if len(plan.joint_trajectory.points) > 0:
                group.execute(plan, wait=True)
                print('hand adjusted')
            else:
                print('no plan result')
                return False
        # 移动到给定位置
        group.set_joint_value_target(pose, True)
        plan_success, plan, planning_time, error_code = group.plan()
        if len(plan.joint_trajectory.points) > 0:
            print('moving')
            group.execute(plan, wait=True)
            return True
        else:
            print('no plan result')
            return False

    def action(self, all_info, pre_result_dict,kalman,yolo):
        raise NotImplementedError