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
from kalman import  Kalman
from pose_predictor import MyModelPredictor
import math
import threading
from scipy.spatial.transform import Rotation as R
from mmpretrain import ImageClassificationInferencer
class TrueMove(TrueBase):
    def __init__(self, group_):
        super(TrueMove, self).__init__(group_)
        self.tf_lock = threading.Lock()
        self.model_predictor = MyModelPredictor('lstm_model_0221.pth', 'data_mean_0221.npy', 'data_std_0221.npy')
        self.stop = False
        self.fail = False
        self.data = []
    
    def get_insert_pose(self,x,y,z):
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

    def plot(self,ft):
        # 设置雷达图参数
        num_vars = ft.shape[0]  # 数据点的数量
        num_categories = ft.shape[1]  # 类别（变量）的数量
        # 计算雷达图的角度
        angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
        angles += angles[:1]  # 完成一个圆圈
        fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))
        # 定义每条图线的标签
        labels = ['F_x', 'F_y', 'F_z', 'T_x', 'T_y', 'T_z']
        # 绘制每一列数据为一条图线
        for j in range(num_categories):
            # 提取每一列数据，形成闭合图线
            this_ft = np.concatenate((ft[:, j], [ft[0, j]]))
            ax.plot(angles, this_ft, '.-', linewidth=1, label=labels[j])
        # 设置图表的布局
        ax.set_thetagrids(np.degrees(angles[:-1]), labels=[f'M{i+1}' for i in range(num_vars)])
        plt.legend(loc='upper right', bbox_to_anchor=(1.1, 1.14))
        plt.title('F/T')
        plt.savefig("radar.jpg",dpi=180)
        plt.close()

    def get_contact_trajectory(self,radius):
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

    def action(self, all_info, pre_result_dict,kalman,yolo,plc,sleeve_type):
        self.inferencer = ImageClassificationInferencer(model='config.py', pretrained='epoch_91.pth', device='cuda')
        # true_x,true_y,true_z = 0.2271,0.9068,0.0725
        # true_x,true_y,true_z = 0.2260,0.6207,0.0724
        # true_x,true_y,true_z = 0.2256,0.8118,0.0716
        true_x,true_y,true_z = 0.167,0.606,0.0728
        # true_x,true_y,true_z = 0.3712,0.0838,0.0744

        print("start to adjust")
        target = self.get_insert_pose(true_x,true_y,true_z)
        
        self.set_arm_pose(self.group, target, self.effector)
        rospy.sleep(0.2)            
        planner = all_info['planner_handler']     
        planner.collect = True         
        insert_trajectory = self.get_contact_trajectory(0)
        for pose in insert_trajectory:
            self.set_arm_pose(self.group, pose, self.effector)
            print ("insert")
            # print (len(planner.wrench_list))
        t = 0
        while (not self.stop) and (t < 100):
            # print (len(planner.wrench_list))               
            sample = np.array(planner.wrench_list[-15:])
            ft = sample[:,8:14]
            self.plot(ft)
            result = self.inferencer('radar.jpg')[0]
            tool_pose = sample[-1, 1:8]
            prediction = self.model_predictor.predict(sample)
            pred_pose = self.model_predictor.combine_poses(tool_pose, prediction)
            ee_pose = self.group.get_current_pose(self.effector).pose
            next_pose = geometry_msgs.msg.Pose()
            next_pose.position.x = ee_pose.position.x
            next_pose.position.y = ee_pose.position.y        
            next_pose.position.z = ee_pose.position.z
            Q1 = [ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]
            Q2 = [pred_pose[3], pred_pose[4], pred_pose[5], pred_pose[6]]
            aligned_quat, deg_dist = self.align_z_axis(Q1, Q2)
            next_pose.orientation.x = aligned_quat[0]
            next_pose.orientation.y = aligned_quat[1]
            next_pose.orientation.z = aligned_quat[2]
            next_pose.orientation.w = aligned_quat[3]
            self.set_arm_pose(self.group, next_pose, self.effector)
            print(deg_dist)
            t += 1
            if result['pred_class'] == "fit" and result['pred_score'] > 0.95:
                # print("置信度："+str(result['pred_score']))
                self.stop = True
                print ('success')
            elif deg_dist > 7.5:
                self.stop = True
                self.fail = True
                print('failure')
        if self.stop == False:
            print('too long')
        rospy.sleep(0.2)
        planner.collect = False
        self.stop = False
        self.fail = False
        print('adjust is end')
        print (t)

        plc.set_effector_star_pos(200)
        rospy.sleep(0.2)
        planner.collect = True
        s = 0
        t = 0
        while (not self.stop) and (t < 8):
            s += 1
            t += 1
            contact_trajectory = self.get_contact_trajectory(0.002)
            print("ready to contact")
            for point in contact_trajectory:
                self.set_arm_pose(self.group, point, self.effector)
                print ("fumble")
            print (len(planner.wrench_list))               
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
            p_pose = geometry_msgs.msg.Pose()
            p_pose.position.x = true_x
            p_pose.position.y = true_y        
            p_pose.position.z = true_z
            p_pose.orientation.x = pred_pose[3]
            p_pose.orientation.y = pred_pose[4]
            p_pose.orientation.z = pred_pose[5]
            p_pose.orientation.w = pred_pose[6]             
            print(next_pose)
            ee_pose.position.z = ee_pose.position.z + 0.005
            self.set_arm_pose(self.group, ee_pose, self.effector)
            if (math.sqrt((next_pose.position.x - true_x) ** 2 + (next_pose.position.y - true_y) ** 2) < 0.002) and (next_pose.position.z - true_z < 0.0025):
                self.stop = True
                print ('success')                                                
                self.set_arm_pose(self.group, next_pose, self.effector)
                rospy.sleep(0.5)
            elif (math.sqrt((next_pose.position.x - true_x) ** 2 + (next_pose.position.y - true_y) ** 2) > 0.0095):
                self.stop = True
                self.fail = True
                print ('failure')
                rospy.is_shutdown(1)
            else:
                self.set_arm_pose(self.group, next_pose, self.effector)
        if self.stop == False:
            print('too long')
            rospy.is_shutdown(1)
        
        print(t)
        print('end')

        plc.set_effector_star_neg(3000)
        planner.motor_speed=plc.read_effector_speed()
        rospy.sleep(0.5)
        planner.motor_speed=plc.read_effector_speed()
        rospy.sleep(0.5)
        planner.motor_speed=plc.read_effector_speed()
        rospy.sleep(0.5)
        planner.motor_speed=plc.read_effector_speed()
        print("speed:",plc.read_effector_speed())
        while True:
            planner.motor_speed=plc.read_effector_speed()
            if planner.cur_torque[2]<0.25:
                break
        planner.collect = False
        rospy.sleep(2)
        plc.set_effector_stop()

        next_pose.position.z += 0.05
        self.set_arm_pose(self.group, next_pose, self.effector)
        rospy.sleep(0.2)
        rospy.is_shutdown(1)