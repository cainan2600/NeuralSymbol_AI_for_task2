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
        # self.model_predictor = MyModelPredictor('lstm_model_0206.pth')
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
        
        # ang_sum = angle * math.pi * 0.5 * (random.random() + 1) / 180
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
            
    
    def publish_bias_frame(self, pose):
        bias_trans = geometry_msgs.msg.TransformStamped()
        bias_trans.header.stamp = rospy.Time.now()
        self.ts=bias_trans.header.stamp
        # self.tf_lock.acquire()
        bias_trans.header.frame_id = "base_link"
        bias_trans.child_frame_id = "bias_screw_frame"
        bias_trans.transform.translation.x = pose.position.x
        bias_trans.transform.translation.y = pose.position.y
        bias_trans.transform.translation.z = pose.position.z
        bias_trans.transform.rotation.x = pose.orientation.x
        bias_trans.transform.rotation.y = pose.orientation.y
        bias_trans.transform.rotation.z = pose.orientation.z
        bias_trans.transform.rotation.w = pose.orientation.w
        # print (bias_trans.transform)
        # (r, p, y) = tf.transformations.euler_from_quaternion([bias_trans.transform.rotation.x, bias_trans.transform.rotation.y, bias_trans.transform.rotation.z, bias_trans.transform.rotation.w])
        # print(r,p,y)
        self.br.sendTransform(bias_trans)
        
    def align_z_axis(self,q1,q2):
        # rot1 = R.from_quat(q1)
        # rot2 = R.from_quat(q2)

        # z_vector = np.array([0,0,1])
        # z1=rot1.apply(z_vector)
        # z2= - rot2.apply(z_vector)
        # print(z2)
        # cross_product = np.cross(z1,z2)
        # dot_product = np.dot(z1,z2)
        # if np.linalg.norm(cross_product) == 0 and dot_product > 0:
        #     return q1
        # elif np.linalg.norm(cross_product) == 0 and dot_product < 0:
        #     rot_axis = np.array([0,1,0]) if z1[0] != 0 else np.array([1,0,0])
        #     return R.from_rotvec(np.pi * rot_axis).as_quat()
        
        # angle = np.arccos(dot_product / (np.linalg.norm(z1) * np.linalg.norm(z2)))       
        # rot_axis = cross_product / np.linalg.norm(cross_product)
        # rot_vector = angle * rot_axis
        # rot_to_align_z = R.from_rotvec(rot_vector)
        # new_rot = rot1 * rot_to_align_z

        
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
        # return aligned_quat,deg_dist

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


    def get_contact_trajectory(self):
        angle = 2.5
        attempt = 4
        trajectory = []
        print('start to calculate contact trajectory')
        ee_pose = self.group.get_current_pose(self.effector).pose
        for i in range(attempt):
            rand_pose = geometry_msgs.msg.Pose()
            rand_pose.position.x = ee_pose.position.x
            rand_pose.position.y = ee_pose.position.y           
            rand_pose.position.z = ee_pose.position.z
            ang_sum = angle * math.pi * 0.5 * (random.random() + 1) / 180
            sig_1 = 1 if random.random() > 0.5 else -1
            ang_1 = ang_sum * random.random()
            sig_2 = 1 if random.random() > 0.5 else -1
            ang_2 = ang_sum - ang_1
            
            ee_orientation = tf.transformations.euler_from_quaternion((ee_pose.orientation.x,ee_pose.orientation.y,ee_pose.orientation.z,ee_pose.orientation.w))
            # q = tf.transformations.quaternion_from_euler(-math.pi + sig_1 * ang_1, sig_2 * ang_2, -0.5*math.pi)
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


    def action(self, all_info, pre_result_dict,kalman,yolo):
        self.inferencer = ImageClassificationInferencer(model='config.py', pretrained='epoch_91.pth', device='cuda')
        # print(self.group.get_current_pose(self.effector).pose)
        # rospy.is_shutdown(1)
        # true_x,true_y,true_z = 0.2271,0.9068,0.0725
        true_x,true_y,true_z = 0.2260,0.6207,0.0724
        # true_x,true_y,true_z = 0.2256,0.8118,0.0716
        # true_x,true_y,true_z = 0.2268,0.4322,0.0722
        # true_x,true_y,true_z = 0.3712,0.0838,0.0744
        deg_threshold = 0.5
        print("start to adjust")
        target = self.get_insert_pose(true_x,true_y,true_z)
        self.set_arm_pose(self.group, target, self.effector)
        rospy.sleep(0.2)            
        planner = all_info['planner_handler']            
        planner.collect = True         
        insert_trajectory = self.get_contact_trajectory()
        for pose in insert_trajectory:
            self.set_arm_pose(self.group, pose, self.effector)
            print ("insert")
            print (len(planner.wrench_list))   
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
        print(Q1)
        Q2 = [pred_pose[3], pred_pose[4], pred_pose[5], pred_pose[6]]
        aligned_quat, deg_dist = self.align_z_axis(Q1, Q2)
        next_pose.orientation.x = aligned_quat[0]
        next_pose.orientation.y = aligned_quat[1]
        next_pose.orientation.z = aligned_quat[2]
        next_pose.orientation.w = aligned_quat[3]
        self.set_arm_pose(self.group, next_pose, self.effector)
        t = 1
        # if deg_dist < deg_threshold:
        if result['pred_class'] == "fit" and result['pred_score'] > 0.95: 
            print("置信度："+str(result['pred_score']))
            self.stop = True
            print ('success')
        print(deg_dist)
        while (not self.stop) and (t < 100):
            t += 1
            print (len(planner.wrench_list))               
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
            # if deg_dist < deg_threshold:
            if result['pred_class'] == "fit" and result['pred_score'] > 0.95:
                print("置信度："+str(result['pred_score']))
                self.stop = True
                print ('success')
            elif deg_dist > 7.5:
                self.stop = True
                self.fail = True
                print('failure')
                # rospy.is_shutdown(1)
        if self.stop == False:
            print('too long')
            # rospy.is_shutdown(1)
        rospy.sleep(0.2)
        planner.collect = False
        # np_wrench_list = np.array(planner.wrench_list)
        # np.savetxt("pose_wrench_"+str(rospy.Time.now().to_sec())+".csv",np_wrench_list, delimiter=",")
        # planner.wrench_list=[]
        # print(self.group.get_current_pose(self.effector).pose)

        # next_pose.position.z += 0.01
        # self.set_arm_pose(self.group, next_pose, self.effector)
        new_target = pre_result_dict["coarse_pose"]
        self.set_arm_pose(self.group, new_target, self.effector)

        # new_target = pre_result_dict["coarse_pose"]
        # self.set_arm_pose(self.group, new_target, self.effector)
        # print(t)
        self.stop = False
        self.fail = False
        
        print('end')
        print (t)

        rospy.is_shutdown(1)