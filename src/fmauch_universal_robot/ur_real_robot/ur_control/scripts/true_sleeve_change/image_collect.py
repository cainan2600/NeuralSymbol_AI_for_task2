#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import importlib
import os
import threading
import csv
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
# from  bolt_position_detector
import copy
import tf2_ros
import geometry_msgs.msg
import traceback
import random
import math

from PIL import Image
# from PIL import Image,ImageDraw
# import numpy as np 
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
from true_base import TrueBase
from scipy.spatial.transform import Rotation as R

class TrueMove(TrueBase):
    def __init__(self, group_):
        super(TrueMove, self).__init__(group_)

    def get_insert_pose(self,x,y,z):
        angle = 10
        print('start to calculate insert pose')
        rand_pose = geometry_msgs.msg.Pose()
        radius = 0.025 * random.random()
        rad = 2 * math.pi * random.random()
        height = 0.025 * (random.random() - 0.5)
        rand_pose.position.x = x + radius * math.cos(rad) - 0.054
        rand_pose.position.y = y + radius * math.sin(rad) - 0.007         
        if rand_pose.position.x < 0.195:
            rand_pose.position.z = z + height
        elif rand_pose.position.x > 0.235:
            rand_pose.position.z = z + abs(height) + 0.035
        else:
            rand_pose.position.z = z + abs(height)
        ang_sum = angle * math.pi * random.random() / 180
        sig_1 = 1 if random.random() > 0.5 else -1
        ang_1 = ang_sum * random.random()
        sig_2 = 1 if random.random() > 0.5 else -1
        ang_2 = ang_sum - ang_1
        q = tf.transformations.quaternion_from_euler(-math.pi + sig_1 * ang_1, sig_2 * ang_2, -0.5*math.pi)
        z = [0,0,0,1]
        _, t_angle = self.align_z_axis(z,q)
        print(180 - t_angle)
        rand_pose.orientation.x = q[0]
        rand_pose.orientation.y = q[1]
        rand_pose.orientation.z = q[2]
        rand_pose.orientation.w = q[3]
        return rand_pose

    def align_z_axis(self,q1,q2):        
        # 提取q1和q2的Z轴向量
        z_axis_q1 = R.from_quat(q1).apply([0, 0, 1])
        z_axis_q2 = R.from_quat(q2).apply([0, 0, 1])
        # 计算目标Z轴向量（q2的Z轴反向）
        target_z_axis = -z_axis_q2
        # 计算从q1的Z轴到目标Z轴的旋转向量
        cross_prod = np.cross(z_axis_q1, target_z_axis)
        dot_prod = np.dot(z_axis_q1, target_z_axis)
        norm_cross_prod = np.linalg.norm(cross_prod)
        if norm_cross_prod < 1e-6:
            # 如果q1的Z轴和目标Z轴已经是反向的，则不需要旋转
            rotation_quat = [0, 0, 0, 1]
        else:
            # 使用叉乘和点乘来计算旋转四元数
            rotation_quat = np.concatenate([cross_prod, [1 + dot_prod]])
            rotation_quat /= np.linalg.norm(rotation_quat)
        # 应用旋转
        aligned_quat = R.from_quat(rotation_quat) * R.from_quat(q1)
        z_new = aligned_quat.apply([0,0,1])
        rad_dist = np.arccos(max(min(np.dot(z_new,[0,0,-1]),1.0),-1.0))
        deg_dist = rad_dist * 180/ np.pi
        return aligned_quat.as_quat(), deg_dist

    def action(self, all_info, pre_result_dict,kalman,yolo,plc,sleeve_type):
        print("start to collect images")
        planner = all_info['planner_handler']
        sample_size = 2000
        with open('image_0327/pose_data.csv','a',newline='') as csvfile:
            datawriter = csv.writer(csvfile,delimiter = ',', quotechar = '|',quoting = csv.QUOTE_MINIMAL)
            for j in range (5):
                if j == 0:
                    true_x,true_y,true_z = 0.1680,0.7964,0.0736 + 0.005
                if j == 1:
                    true_x,true_y,true_z = 0.1675,0.7011,0.0732 + 0.005
                if j == 2:
                    true_x,true_y,true_z = 0.1671,0.6060,0.0742 + 0.005
                if j == 3:
                    true_x,true_y,true_z = 0.1673,0.5108,0.0736 + 0.005
                if j == 4:
                    true_x,true_y,true_z = 0.1670,0.4166,0.0738 + 0.005            
                for i in range(sample_size):
                    print(j+1,i)
                    target = self.get_insert_pose(true_x,true_y,true_z)
                    self.set_arm_pose(self.group, target, self.effector)
                    print('finish')
                    rospy.sleep(0.2)
                    img_path = str(j+1)+'_'+str(i+1)+'.png'
                    ee_pose = self.group.get_current_pose(self.effector).pose
                    pose_data = [img_path,ee_pose.position.x,ee_pose.position.y,ee_pose.position.z,ee_pose.orientation.x,ee_pose.orientation.y,ee_pose.orientation.z,ee_pose.orientation.w]
                    planner = all_info['planner_handler']          
                    latest_infos = planner.get_latest_infos()      
                    rgb_img = latest_infos['rgb_img']
                    dep_img = latest_infos['depth_img']
                    cv2.imwrite('image_0327/rgb_img/'+ str(img_path),rgb_img)
                    cv2.imwrite('image_0327/dep_img/'+ str(img_path),dep_img)
                    datawriter.writerow(pose_data)
                    rospy.sleep(0.2)
