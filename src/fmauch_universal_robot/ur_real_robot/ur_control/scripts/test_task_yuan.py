#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import threading
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
import copy
import moveit_commander
import math
import json
import socket
import tf2_ros
import geometry_msgs.msg
import math
import select, termios, tty
import socket
import pickle
import struct
from queue import Queue
from itertools import chain
from geometry_msgs.msg import WrenchStamped, Pose,TransformStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
from sensor_msgs.msg import JointState
from MODBUS_client import MODBUS_control
from true_insert import TrueInsert
# from image_collect import TrueInsert
from true_disassemble import TrueDisassemble
from true_fumble import TrueFumble
from true_search_and_reinsert import TrueSearchAndReinsert
from true_action import PrimAction
from mmpretrain import ImageClassificationInferencer


class TSTPlanner:
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        # self.camera_name = camera_name
        # self.rgb_topic = rgb_topic
        # self.depth_topic = depth_topic
        # self.camera_info_topic = camera_info_topic
        # self.bolt_trans_topic = '/NSPlanner/bolt_trans'
        # self.pose = None
        # self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        # self.br = tf2_ros.TransformBroadcaster()

        # Have we recieved camera_info and image yet?
        # self.ready_ = False
        # self.bridge = CvBridge()
        # self.camera_model = image_geometry.PinholeCameraModel()
        # rospy.loginfo(
        #     'Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        # print('')

        # q = 1
        # self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        # self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        # self.sub_camera_info = rospy.Subscriber(camera_info_topic, CameraInfo, self.cam_info_cb)
        # self.camera_model_ready = False
        # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth],
        #                                                        queue_size=30, slop=0.2)

        # self.tss.registerCallback(self.callback)

        # moveit_commander.roscpp_initialize(sys.argv)
        # self.group = moveit_commander.MoveGroupCommander("manipulator")
        # self.effector = self.group.get_end_effector_link()
        # print("self.effector:",self.effector)

        # self.stage={'above_screw':True,'near_screw':False,'target_aim':False, 'target_clear':True,'clamped':False,'part_clamped':False,'disassembled':False}        
        # self.group.set_planner_id("RRTConnectkConfigDefault")

        # self.plc_control=MODBUS_control()

        #初始化stage
        # self.insert_prim=TrueInsert(self.group)
        # self.fumble_prim=TrueFumble(self.group)
        # self.search_prim=TrueSearchAndReinsert(self.group)
        # self.disassemble_prim=TrueDisassemble(self.group)
        # self.prims = {'insert': self.insert_prim,
        #               'fumble': self.fumble_prim,
        #               'search_and_reinsert': self.search_prim,
        #               'disassemble':self.disassemble_prim}
        # self.action = 'sleep'
        # self.all_infos = {}
        # self.ret_dict = {}
        self.bolt_pose = None
        # self.all_infos_lock = threading.Lock()
        # self.prim_thread = threading.Thread(target=self.do_action)
        # self.prim_execution = True
        self.shut_down = False
        # self.prim_thread.start()

        # np_verification
        # self.np_checker = ImageClassificationInferencer(model='mobilevit-xxsmall_8xb128_contact.py', pretrained='epoch_90.pth', device='cuda')

        #把需要发送的数据保存为变量
        # self.tool_pose=None
        # self.joint_state=None
        # self.pose_list=None
        # self.curr_ob_pose=None
        # self.next_ob_pose=None
        # self.ori_force=None        
        # self.ori_torque=None
        # self.cur_force=None
        # self.cur_torque=None
        # self.wrench_list=[]
        # self.sleeve_type=None
        # self.sleeve_type="hex_bolt_8"
        # self.aim_bolt_type=None
        # self.motor_speed=None
        # self.collect=True

        #发布坐标,力和力矩
        # self.sub_wrench=rospy.Subscriber("/ft_wrench", WrenchStamped, self.force_callback)
        # # self.sub_wrench=rospy.Subscriber("/wrench", WrenchStamped, self.force_callback)
        # self.sub_joint=rospy.Subscriber("/joint_states",JointState,self.joint_callback) 
        # self.tool_pose_thread=threading.Thread(target=self.tool_pose_update)
        # self.tool_pose_thread.start()

    # def force_callback(self,msg): 
    #     try:
    #         x_force=msg.wrench.force.x
    #         y_force=msg.wrench.force.y
    #         z_force=msg.wrench.force.z
    #         x_torque=msg.wrench.torque.x
    #         y_torque=msg.wrench.torque.y
    #         z_torque=msg.wrench.torque.z            
    #         raw_force=np.array([x_force,y_force,z_force])
    #         raw_torque=np.array([x_torque,y_torque,z_torque])
    #         if (self.ori_force is None) and (self.ori_torque is None):
    #             self.ori_force = raw_force
    #             self.ori_torque = raw_torque
    #             print ('original wrench collected')
    #         elif self.collect == True:
    #             self.t_cur_force = raw_force-self.ori_force
    #             self.t_cur_torque = raw_torque-self.ori_torque
    #             self.cur_force = self.t_cur_force.tolist()
    #             self.cur_torque = self.t_cur_torque.tolist()
    #     except Exception as err:
    #         print("Exception happen in message call back:", err)
    
    # def joint_callback(self,msg): 
    #     try:
    #         joint_state = np.array(msg.position) * 180 / np.pi
    #         self.joint_state = joint_state.tolist()
    #     except Exception as err:
    #         print("Exception happen in message call back:", err)

    # def tool_pose_update(self):
    #     rate = rospy.Rate(15)
    #     while not (rospy.is_shutdown() and self.prim_execution):
    #         try:
    #             pose = self.group.get_current_pose(self.effector).pose
    #             if (not pose is None) and self.collect:
    #                 self.t_tool_pose = np.array([rospy.Time.now().to_sec(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    #                 pose_wrench = np.concatenate((self.t_tool_pose,self.t_cur_force, self.t_cur_torque)).reshape([1,14])
    #                 self.wrench_list.append(pose_wrench.tolist()[0])
    #         except Exception as e:
    #             print('Error:',e)

    #         rate.sleep()

    # def auto_plan(self,original_stage):
        # print ('start to plan')
        # ori_stage=original_stage

        # #建立动作原语集
        # insert=PrimAction('insert')
        # fumble=PrimAction('fumble')
        # search=PrimAction('search_and_reinsert')
        # disassemble=PrimAction('disassemble')
        # prim_list=(insert,fumble,search,disassemble)

        # #基于FIFO的规划生成方法
        # pathQueue=Queue(0)
        # pathQueue.put([ori_stage,[]])
        # plan_is_end=False
        # while not plan_is_end:
        #     tmp_pair=pathQueue.get()
            # tmp_stage=tmp_pair[0]
            # tmp_path=tmp_pair[1]
            # if tmp_stage['disassembled']==True:
            #     pathQueue.put(tmp_pair)
            #     plan_is_end=True
            # else:
            #     for primi in prim_list:
            #         if primi.able(tmp_stage)==True:
            #             new_stage=primi.action(tmp_stage)
            #             new_path=[]
            #             for n in tmp_path:
            #                 new_path.append(n)
            #             new_path.append(primi.prim)
            #             pathQueue.put([new_stage,new_path])
        # path_list=[]
        # while not pathQueue.empty():
        #     path=pathQueue.get()
        #     path_list.append(path[1])        

        # #筛选出所有最短步数的规划方案
        # min_step=100
        # for path in path_list:
        #     if len(path)<min_step:
        #         min_step=len(path)
        # path_list=[i for i in path_list if len(i)==min_step]
        # print (path_list[0])
        # return path_list[0]

    # def start(self,  pose):
    #     # self.plc_control.set_return_zero()
    #     if self.action != 'sleep':
    #         print("Please start after previous task was done!")
    #         return False
    #     else:
    #         self.stage['target_aim']= True
    #         self.action = 'start'
    #         return True

    def do_action(self):
        #执行动作
        insert=PrimAction('insert')
        fumble=PrimAction('fumble')
        search=PrimAction('search_and_reinsert')
        disassemble=PrimAction('disassemble')
        prim_dict={'insert':insert,'fumble':fumble,'search_and_reinsert':search,'disassemble':disassemble}
        
        while self.prim_execution:
            if self.action== 'sleep':
                rospy.sleep(1)
                continue
            else:
                if self.action == 'start':
                    print('action==start do auto_plan')
                    step_list = self.auto_plan(self.stage)
                    i = 0
                    self.action = step_list[i]
                    print(self.action)
                if self.all_infos_lock.acquire():
                    infos = copy.deepcopy(self.all_infos)
                    self.all_infos.clear()
                    self.all_infos_lock.release()
                    if self.action in prim_dict.keys():
                        #检测pre是否满足
                        pre_is_ok = True
                        pic_state = infos['rgb_img']
                        size = 320
                        left = 1440
                        top = 560
                        pic_state = pic_state[top:top + size, left:left + size]        
                        cv2.imwrite('np_check.png',pic_state)
                        result = self.np_checker('np_check.png')[0]
                        print(result)
                        if result['pred_class'] == "part_clamped":
                            self.stage['part_clamped']=True
                            self.stage['clamped']=False
                        if result['pred_class'] == "clamped":
                            self.stage['part_clamped']=False
                            self.stage['clamped']=True
                        if result['pred_class'] == "not_clamped":
                            self.stage['part_clamped']=False
                            self.stage['clamped']=False        
                        for pre in (prim_dict[self.action]).pre:
                            if not self.stage[pre]==(prim_dict[self.action].pre)[pre]:
                                pre_is_ok=False
                                print("Precondition not satisfied")
                                break
                        if pre_is_ok==True:
                            prim = self.prims[self.action]
                            #execute primitive       
                            infos['planner_handler']=self
                            self.ret_dict = prim.action(infos, self.ret_dict,self.plc_control)
                            print(self.action+"_is_finished")

                            #update effect
                            for eff in (prim_dict[self.action]).eff:
                                self.stage[eff]=(prim_dict[self.action].eff)[eff]
                            i = i + 1
                            if self.action=='disassemble':
                                print('Task is finished')
                                rospy.is_shutdown(1)
                        else:
                            #若pre不满足，重新生成规划方案
                            step_list=self.auto_plan(self.stage)
                            i=0
                            self.action=step_list[i]
                    rospy.sleep(0.5)

    def get_latest_infos(self):
        print('get_latest_infos')
        rospy.sleep(0.1)
        if self.all_infos_lock.acquire():
            infos = copy.deepcopy(self.all_infos)
            self.all_infos.clear()
            self.all_infos_lock.release()
            return infos
        else:
            return None

    def cam_info_cb(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_model_ready = True
        self.sub_camera_info.unregister()

    def callback(self, rgb_msg, depth_msg):
        try:
            if not self.camera_model_ready:
                print("camera info is not ready")
                return
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            ts = rospy.Time.now()
            if self.all_infos_lock.acquire():
                self.all_infos = {'rgb_img': img, 'depth_img': depth_img,
                                  'camera_model': self.camera_model, 'timestamp': ts}
                self.all_infos_lock.release()

        except Exception as err:
            print("exception happen in message call back:", err)

    def __del__(self):
        self.prim_execution = False
        self.prim_thread.join()
    
    def print_pose(self,pose):
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(q)
        print ('%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
            (self.effector, pose.position.x, pose.position.y, pose.position.z, \
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
            rpy[0], rpy[1], rpy[2]))

if __name__ == '__main__':

    try:
        rospy.init_node('tstplanner-moveit', anonymous=True)

        planner = TSTPlanner('camera', '/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', '/camera/color/camera_info')
            
        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, -0.5*math.pi)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = 0.17
        pose_target.position.y = 0.7
        pose_target.position.z = 0.2
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        planner.start(pose_target)

        while not rospy.is_shutdown():
            rospy.spin()
        
        del planner
        
    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()