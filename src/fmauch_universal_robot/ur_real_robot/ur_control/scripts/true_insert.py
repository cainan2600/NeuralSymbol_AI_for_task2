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
from rigid_transform_3D import rigid_transform_3D
from pose_predictor import MyModelPredictor
import math
import threading
from scipy.spatial.transform import Rotation as R
from mmpretrain import ImageClassificationInferencer
class TrueInsert(TrueBase):
    def __init__(self, group_):
        super(TrueInsert, self).__init__(group_)
        self.tf_lock = threading.Lock()
        self.stop = False
        self.fail = False
        self.data = []
    
    def get_insert_pose(self,x,y,z):

        '''
        在读取的位置位姿上作随机处理，模拟插入失败
        '''

        angle = 0
        print('start to calculate insert pose')

        rand_pose = geometry_msgs.msg.Pose()

        # 生成三个随机数，
        radius = 0.01 * random.random()
        rad = 2 * math.pi * random.random()
        height = 0.005 * (random.random() - 0.5)

        # 当前位置参数加上随机数，模拟插入失败的情况
        rand_pose.position.x = x + radius * math.cos(rad)
        rand_pose.position.y = y + radius * math.sin(rad)       
        rand_pose.position.z = z + height

        # ang_sum是一个随机角度，这里为0
        ang_sum = angle * math.pi * random.random() / 180
        sig_1 = 1 if random.random() > 0.5 else -1
        ang_1 = ang_sum * random.random()
        sig_2 = 1 if random.random() > 0.5 else -1
        ang_2 = ang_sum - ang_1
        # 将欧拉角转换为四元数
        q = tf.transformations.quaternion_from_euler(-math.pi + sig_1 * ang_1, sig_2 * ang_2, -0.5*math.pi)

        # 定义一个指向z轴的四元数
        z = [0,0,0,1]
        # 对齐Z轴，并返回旋转角度
        _, t_angle = self.align_z_axis(z,q)
        print(180 - t_angle)
        # 将计算得到的四元数 q 分别赋值给 rand_pose 的方向（orientation）的四个分量
        rand_pose.orientation.x = q[0]
        rand_pose.orientation.y = q[1]
        rand_pose.orientation.z = q[2]
        rand_pose.orientation.w = q[3]
        return rand_pose        
        
    def align_z_axis(self,q1,q2):

        '''
        计算并应用一个旋转，使得 q1 的Z轴对齐到 q2 的反向Z轴，同时返回对齐后的四元数和角度距离
        '''

        # 提取q1和q2的Z轴向量，将四元数旋转应用到Z轴向量 [0, 0, 1] 上，得到旋转后的Z轴方向
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

    def action(self, all_info, pre_result_dict,plc):
        # 输入目标螺钉位置
        true_x,true_y,true_z = 0.169,0.321,0.077
        print("start to insert")
        target = self.get_insert_pose(true_x,true_y,true_z)
        # 使机械臂运动到该位置，详细见true_base
        self.set_arm_pose(self.group, target, self.effector)
        # 获取当前位姿信息
        ee_pose = self.group.get_current_pose(self.effector).pose
        print(ee_pose)
        # 设定末端执行器旋转速度
        plc.set_effector_star_pos(200)
        rospy.sleep(0.2)
        return {'success': True}