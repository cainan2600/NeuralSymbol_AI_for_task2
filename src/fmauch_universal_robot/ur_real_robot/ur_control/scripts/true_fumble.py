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
class TrueFumble(TrueBase):
    def __init__(self, group_):
        super(TrueFumble, self).__init__(group_)
        self.tf_lock = threading.Lock()
        # 使用LSTM预测末端执行器位姿
        self.model_predictor = MyModelPredictor('lstm_model_0221.pth', 'data_mean_0221.npy', 'data_std_0221.npy')
        self.stop = False
        self.fail = False
        self.data = []

    def get_contact_trajectory(self,radius):

        '''
        生成随机四个点位的摸索轨迹
        '''

        angle = 2.5
        # 四个点
        attempt = 4
        trajectory = []
        print('start to calculate contact trajectory')
        # 获取当前位置、位姿信息
        ee_pose = self.group.get_current_pose(self.effector).pose
        for i in range(attempt):
            rand_pose = geometry_msgs.msg.Pose()
            # 随机扩大范围摸索
            rad = 2 * math.pi * random.random()
            rand_pose.position.x = ee_pose.position.x + radius * 0.5 * (random.random() + 1) * math.cos(rad)
            rand_pose.position.y = ee_pose.position.y + radius * 0.5 * (random.random() + 1) * math.sin(rad)
            rand_pose.position.z = ee_pose.position.z + radius * (random.random() - 0.5)
            ang_sum = angle * math.pi * 0.5 * (random.random() + 1) / 180
            sig_1 = 1 if random.random() > 0.5 else -1
            ang_1 = ang_sum * random.random()
            sig_2 = 1 if random.random() > 0.5 else -1
            ang_2 = ang_sum - ang_1
            # 四元数转换为欧拉角
            ee_orientation = tf.transformations.euler_from_quaternion((ee_pose.orientation.x,ee_pose.orientation.y,ee_pose.orientation.z,ee_pose.orientation.w))
            # 欧拉角转换为四元数
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
        # 可以访问planner中定义的实例
        planner = all_info['planner_handler']     
        planner.collect = True
        # 获取当前位置、位姿信息
        ee_pose = self.group.get_current_pose(self.effector).pose
        # 调用轨迹获取函数
        contact_trajectory = self.get_contact_trajectory(0.002)
        print("ready to contact")
        for point in contact_trajectory:
            # 利用MOVEIT实现机械臂按照轨迹运动
            self.set_arm_pose(self.group, point, self.effector)
            print ("fumble")
        print (len(planner.wrench_list))
        # 获取当前位置、位姿、力、力矩信息               
        sample = np.array(planner.wrench_list[-15:])
        # 切割出当前位置位姿
        tool_pose = sample[-1, 1:8]
        # 利用力、力矩信息通过LSTM预测位置位姿
        prediction = self.model_predictor.predict(sample)
        # 结合当前和预测位姿为目标位置位姿，计算出目标世界坐标系下的末端执行器位置位姿
        pred_pose = self.model_predictor.combine_poses(tool_pose, prediction)
        
        next_pose = geometry_msgs.msg.Pose()
        next_pose.position.x = pred_pose[0]
        next_pose.position.y = pred_pose[1]        
        next_pose.position.z = pred_pose[2]
        next_pose.orientation.x = ee_pose.orientation.x
        next_pose.orientation.y = ee_pose.orientation.y
        next_pose.orientation.z = ee_pose.orientation.z
        next_pose.orientation.w = ee_pose.orientation.w
        print(next_pose)
        # 运动到目标位置
        self.set_arm_pose(self.group, next_pose, self.effector)

        print('end')
        rospy.sleep(0.2)
        return {'success': True}