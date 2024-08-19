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
class TrueMove(TrueBase):
    def __init__(self, group_):
        super(TrueMove, self).__init__(group_)
        self.tf_lock = threading.Lock()
        self.model_predictor = MyModelPredictor('lstm_model_0206.pth', 'data_mean_0206.npy', 'data_std_0206.npy')
        # self.model_predictor = MyModelPredictor('lstm_model_0206.pth')
        self.stop = False
        self.fail = False
        self.data = []

    def get_contact_trajectory(self,time):
        radius = 0.002
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
            q = tf.transformations.quaternion_from_euler(-math.pi + sig_1 * ang_1, sig_2 * ang_2, -0.5*math.pi)
            rand_pose.orientation.x = q[0]
            rand_pose.orientation.y = q[1]
            rand_pose.orientation.z = q[2]
            rand_pose.orientation.w = q[3]
            if not rand_pose is None:
                trajectory.append(rand_pose)
        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory
    
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

    def calculate_distance(self, pred_pose, ee_pose):
        pos_distance = math.sqrt(math.pow(float(ee_pose.position.x) - float(pred_pose.position.x), 2) + math.pow(float(ee_pose.position.y) - float(pred_pose.position.y), 2) + math.pow(float(ee_pose.position.z) - float(pred_pose.position.z), 2))
        q1 = [pred_pose.orientation.x, pred_pose.orientation.y, pred_pose.orientation.z, pred_pose.orientation.w]
        q2 = [ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]
        rot1 = R.from_quat(q1)
        rot2 = R.from_quat(q2)
        v_z= np.array([0,0,1])
        z1 = - rot1.apply(v_z)
        z2 = rot2.apply(v_z)
        rad_distance = np.arccos(max(min(np.dot(z1, z2),1.0),-1.0))
        deg_distance = rad_distance * 180/ np.pi
        return pos_distance,deg_distance
        
    def align_z_axis(self,q1,q2):
        rot1 = R.from_quat(q1)
        rot2 = R.from_quat(q2)

        z_vector = np.array([0,0,1])
        z1=rot1.apply(z_vector)
        z2= - rot2.apply(z_vector)
        rad_dist = np.arccos(max(min(np.dot(z2,-z_vector),1.0),-1.0))
        deg_dist = rad_dist * 180/ np.pi

        cross_product = np.cross(z1,z2)
        dot_product = np.dot(z1,z2)

        if np.linalg.norm(cross_product) == 0 and dot_product > 0:
            return q1
        elif np.linalg.norm(cross_product) == 0 and dot_product < 0:
            rot_axis = np.array([0,1,0]) if z1[0] != 0 else np.array([1,0,0])
            return R.from_rotvec(np.pi * rot_axis).as_quat()
        
        angle = np.arccos(dot_product / (np.linalg.norm(z1) * np.linalg.norm(z2)))
        rot_axis = cross_product / np.linalg.norm(cross_product)
        rot_vector = angle * rot_axis
        rot_to_align_z = R.from_rotvec(rot_vector)
        new_rot = rot1 * rot_to_align_z
        aligned_quat = new_rot.as_quat()
        return aligned_quat,deg_dist

    def action(self, all_info, pre_result_dict,kalman,yolo):
        # print(self.group.get_current_pose(self.effector).pose)
        # rospy.is_shutdown(1)
        test_time = 50
        success_time = 0
        failure_time = 0
        # true_x,true_y,true_z = 0.2271,0.9068,0.0725
        true_x,true_y,true_z = 0.2260,0.6220,0.0724
        # true_x,true_y,true_z = 0.2256,0.8118,0.0716
        # true_x,true_y,true_z = 0.2268,0.4322,0.0722
        # true_x,true_y,true_z = 0.3712,0.0838,0.0744

        total_fumble = 0

        for i in range (test_time):
            print("start to insert")
            radius = 0.005 * 0.5 * (random.random() + 1)
            rad = 2 * math.pi * random.random()
            height = 0.005 * (random.random() - 0.5)
            target = geometry_msgs.msg.Pose()
            target.position.x = true_x + radius * math.cos(rad)
            target.position.y = true_y + radius * math.sin(rad)       
            target.position.z = true_z + height
            # q = tf.transformations.quaternion_from_euler(-math.pi, 0, -0.5*math.pi)
            # target.orientation.x = q[0]
            # target.orientation.y = q[1]
            # target.orientation.z = q[2]
            # target.orientation.w = q[3]
            angle = 2.5
            ang_sum = angle * math.pi * 0.5 * (random.random() + 1) / 180
            sig_1 = 1 if random.random() > 0.5 else -1
            ang_1 = ang_sum * random.random()
            sig_2 = 1 if random.random() > 0.5 else -1
            ang_2 = ang_sum - ang_1
            q = tf.transformations.quaternion_from_euler(-math.pi + sig_1 * ang_1, sig_2 * ang_2, -0.5*math.pi)
            target.orientation.x = q[0]
            target.orientation.y = q[1]
            target.orientation.z = q[2]
            target.orientation.w = q[3]
            self.set_arm_pose(self.group, target, self.effector)
            rospy.sleep(0.2)
            planner = all_info['planner_handler']
            planner.collect = True
            s = 0
            t = 0
            while (not self.stop) and (t < 5):
                s += 1
                t += 1
                contact_trajectory = self.get_contact_trajectory(s)
                print("ready to contact")
                # planner.collect = True
                # rospy.sleep(0.1)
                for point in contact_trajectory:
                    self.set_arm_pose(self.group, point, self.effector)
                    print ("fumble")
                    # rospy.sleep(0.05)
                # print (planner.wrench_list.shape[0])
                print (len(planner.wrench_list))               
                sample = np.array(planner.wrench_list[-15:])
                # planner.collect = False
                # raw_sample = np.array(planner.wrench_list[-15:])
                # sample = raw_sample[-10:]
                # self.data.append(raw_sample)
                tool_pose = sample[-1, 1:8]
                prediction = self.model_predictor.predict(sample)
                pred_pose = self.model_predictor.combine_poses(tool_pose, prediction)
                ee_pose = self.group.get_current_pose(self.effector).pose
                next_pose = geometry_msgs.msg.Pose()
                next_pose.position.x = pred_pose[0]
                next_pose.position.y = pred_pose[1]        
                next_pose.position.z = pred_pose[2]
                # next_pose.orientation.x = q[0]
                # next_pose.orientation.y = q[1]
                # next_pose.orientation.z = q[2]
                # next_pose.orientation.w = q[3]
                Q1 = [ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]
                Q2 = [pred_pose[3], pred_pose[4], pred_pose[5], pred_pose[6]]
                aligned_quat, deg_dist = self.align_z_axis(Q1, Q2)
                next_pose.orientation.x = aligned_quat[0]
                next_pose.orientation.y = aligned_quat[1]
                next_pose.orientation.z = aligned_quat[2]
                next_pose.orientation.w = aligned_quat[3]               
                # pos_dist, deg_dist = self.calculate_distance(next_pose, ee_pose)
                # print(pos_dist)
                print(deg_dist)
                print(next_pose)
                ee_pose.position.z = ee_pose.position.z + 0.005
                self.set_arm_pose(self.group, ee_pose, self.effector)
                # rospy.sleep(0.1)
                if (math.sqrt((next_pose.position.x - true_x) ** 2 + (next_pose.position.y - true_y) ** 2) < 0.002) and (next_pose.position.z - true_z < 0.0025) and deg_dist < 1.5:
                    self.stop = True
                    success_time += 1
                    print ('success')                                                
                    self.set_arm_pose(self.group, next_pose, self.effector)
                    total_fumble += t
                # rospy.sleep(0.2)
                # ee_pose = self.group.get_current_pose(self.effector).pose
                elif (math.sqrt((next_pose.position.x - true_x) ** 2 + (next_pose.position.y - true_y) ** 2) > 0.0095):
                    self.stop = True
                    self.fail = True
                    failure_time += 1
                    print ('failure')
                else:
                    self.set_arm_pose(self.group, next_pose, self.effector)
            if self.stop == False:
                print('too long')
            planner.collect = False
            # if self.fail:
            #     np.savetxt("pose_wrench_"+str(rospy.Time.now().to_sec())+".csv",planner.wrench_list, delimiter=",")
            print(i)            
            next_pose.position.z = next_pose.position.z + 0.01
            self.set_arm_pose(self.group, next_pose, self.effector)
            # rospy.sleep(0.2)
            self.stop = False
            self.fail = False
        average_fumble = total_fumble / success_time
        print('end')
        print (success_time,failure_time)
        print (average_fumble)
        
        np_wrench_list = np.array(planner.wrench_list)
        # np_wrench_list = np.concatenate((self.data), axis=0)
        np.savetxt("pose_wrench_"+str(rospy.Time.now().to_sec())+".csv",np_wrench_list, delimiter=",") 

        # np.savetxt("pose_wrench_"+str(rospy.Time.now().to_sec())+".csv",planner.wrench_list, delimiter=",") 
        rospy.is_shutdown(1)
        return {'success': True}