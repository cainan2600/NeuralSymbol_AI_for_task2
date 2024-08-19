#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from true_base import TrueBase
import math
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import tf
import rospy
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import copy
import tf2_ros
import traceback
import random
from pose_predictor import MyModelPredictor
import math
import threading
from scipy.spatial.transform import Rotation as R
from mmpretrain import ImageClassificationInferencer
class TrueSearchAndReinsert(TrueBase):

    '''主要修改这个类完成第三种情况，以下仅供参考'''

    def __init__(self, group_):
        super(TrueSearchAndReinsert, self).__init__(group_)
        self.tf_lock = threading.Lock()
        # 导入LSTM预测模型
        self.model_predictor = MyModelPredictor('lstm_model_0221.pth', 'data_mean_0221.npy', 'data_std_0221.npy')
        self.stop = False
        self.fail = False
        self.data = []
    
    def get_insert_pose(self,x,y,z):

        '''这部分是true_insert的，仅供参考'''

        angle = 6
        print('start to calculate insert pose')
        rand_pose = geometry_msgs.msg.Pose()
        radius = 0.008 * random.random()
        rad = 2 * math.pi * random.random()
        height = 0.005 * (random.random() - 0.5)
        rand_pose.position.x = x + radius * math.cos(rad)
        rand_pose.position.y = y + radius * math.sin(rad)       
        rand_pose.position.z = z + height
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


        '''这部分是true_insert的，仅供参考'''

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

    def get_contact_trajectory(self,radius):

        '''
        此处与true_fumble相同，旨在参考，注意题目
        '''

        angle = 2.5
        attempt = 4
        trajectory = []
        print('start to calculate contact trajectory')
        ee_pose = self.group.get_current_pose(self.effector).pose
        for i in range(attempt):
            rand_pose = geometry_msgs.msg.Pose()
            rad = 2 * math.pi * random.random()
            rand_pose.position.x = ee_pose.position.x + radius * 0.5 * (random.random() + 1) * math.cos(rad)
            rand_pose.position.y = ee_pose.position.y + radius * 0.5 * (random.random() + 1) * math.sin(rad)
            rand_pose.position.z = ee_pose.position.z + radius * (random.random() - 0.5)
            ang_sum = angle * math.pi * 0.5 * (random.random() + 1) / 180
            sig_1 = 1 if random.random() > 0.5 else -1
            ang_1 = ang_sum * random.random()
            sig_2 = 1 if random.random() > 0.5 else -1
            ang_2 = ang_sum - ang_1
            ee_orientation = tf.transformations.euler_from_quaternion((ee_pose.orientation.x,ee_pose.orientation.y,ee_pose.orientation.z,ee_pose.orientation.w))
            q = tf.transformations.quaternion_from_euler(ee_orientation[0] + sig_1 * ang_1, ee_orientation[1] + sig_2 * ang_2, ee_orientation[2])
            rand_pose.orientation.x = q[0]
            rand_pose.orientation.y = q[1]
            rand_pose.orientation.z = q[2]
            rand_pose.orientation.w = q[3]
            if not rand_pose is None:
                trajectory.append(rand_pose)
        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory

    def action(self, all_info, pre_result_dict,plc):

        '''
        和true_fumble一样，仅供参考
        '''
    
        # 使用planner中的实例，及读取力、力矩信息。摸索的时候根据力判断是否碰撞到
        planner = all_info['planner_handler']     
        planner.collect = True
        # 获取轨迹
        contact_trajectory = self.get_contact_trajectory(0.002)
        print("ready to contact")
        # 按轨迹运动
        for point in contact_trajectory:
            self.set_arm_pose(self.group, point, self.effector)
            print ("fumble")
        print (len(planner.wrench_list))
        # 读取力、力矩信息               
        sample = np.array(planner.wrench_list[-15:])
        tool_pose = sample[-1, 1:8]
        prediction = self.model_predictor.predict(sample)
        pred_pose = self.model_predictor.combine_poses(tool_pose, prediction)
        ee_pose = self.group.get_current_pose(self.effector).pose
        
        next_pose = geometry_msgs.msg.Pose()
        next_pose.position.x = pred_pose[0]
        next_pose.position.y = pred_pose[1]        
        next_pose.position.z = pred_pose[2]
        next_pose.orientation.x = ee_pose.orientation.x
        next_pose.orientation.y = ee_pose.orientation.y
        next_pose.orientation.z = ee_pose.orientation.z
        next_pose.orientation.w = ee_pose.orientation.w
        print(next_pose)
        self.set_arm_pose(self.group, next_pose, self.effector)

        print('end')
        rospy.sleep(0.2)
        return {'success': True}