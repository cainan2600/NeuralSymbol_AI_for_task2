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
from image_collect import TrueInsert
from true_disassemble import TrueDisassemble
from true_fumble import TrueFumble
from true_search_and_reinsert import TrueSearchAndReinsert
from true_action import PrimAction
from mmpretrain import ImageClassificationInferencer


class TSTPlanner:
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):
        # 初始化planner时应传入相机名称，rgb话题，深度话题，相机话题
        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        # 打印相机相关信息
        rospy.loginfo('Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        # print('')
        # 生成相机模型
        self.camera_model = image_geometry.PinholeCameraModel()

        # 将话题名定义为变量
        self.bolt_trans_topic = '/NSPlanner/bolt_trans'

        # 记录当前位置信息
        self.pose = None

        # 发布坐标变换（）
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10) # 实例化发布对象
        self.br = tf2_ros.TransformBroadcaster() # 创建TF广播器

        self.ready_ = False
        # 用于处理，转换图像格式
        self.bridge = CvBridge()
        
        # 以下是一些订阅方和客户端的实例化/由realshen（相机）发布
        q = 1
        self.camera_model_ready = False
        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        self.sub_camera_info = rospy.Subscriber(camera_info_topic, CameraInfo, self.cam_info_cb)
        # ros中订阅多个topic时同步时间戳
        self.tss = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=30,
            slop=0.2)
        # 使用registerCallback()方法注册多个回调，他们将会按照注册的顺序依次进行调用
        self.tss.registerCallback(self.callback)

        # 初始化
        moveit_commander.roscpp_initialize(sys.argv)
        # 读出末端执行器名词
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        # 选择运动链
        self.effector = self.group.get_end_effector_link()
        print("self.effector:",self.effector)

        # 这样写是定义了实例化后的默认谓词状态
        self.stage={'above_screw':True,
            'near_screw':False,
            'target_aim':False, 
            'target_clear':True,
            'clamped':False,
            'part_clamped':False,
            'disassembled':False}        
        # 选择规划算法
        self.group.set_planner_id("RRTConnectkConfigDefault")
        # 运行PLC控制
        self.plc_control=MODBUS_control()

        #初始化stage
        self.insert_prim=TrueInsert(self.group)
        self.fumble_prim=TrueFumble(self.group)
        self.search_prim=TrueSearchAndReinsert(self.group)
        self.disassemble_prim=TrueDisassemble(self.group)
        self.prims = {'insert': self.insert_prim,
                      'fumble': self.fumble_prim,
                      'search_and_reinsert': self.search_prim,
                      'disassemble':self.disassemble_prim}

        self.action = 'sleep' # sleep表示暂时不能执行任何动作，没有粗位置
        self.all_infos = {}
        self.ret_dict = {}

        # 实例化已训练的网络模型（mmpretrain），训练好的权重以及采用cuda加速运行
        self.np_checker = ImageClassificationInferencer(model='mobilevit-xxsmall_8xb128_contact.py', pretrained='epoch_90.pth', device='cuda')

        #把需要发送的数据保存为变量
        self.prim_execution = True
        self.tool_pose=None
        self.joint_state=None
        self.ori_force=None        
        self.ori_torque=None
        self.cur_force=None
        self.cur_torque=None
        self.wrench_list=[]
        self.collect=True
        # 未用到
        self.bolt_pose = None
        self.shut_down = False
        self.pose_list=None
        self.curr_ob_pose=None
        self.next_ob_pose=None
        self.sleeve_type=None
        self.sleeve_type="hex_bolt_8"
        self.aim_bolt_type=None
        self.motor_speed=None

        #订阅坐标,力和力矩，并调用回调函数处理信息
        self.sub_wrench=rospy.Subscriber("/ft_wrench", WrenchStamped, self.force_callback)
        self.sub_joint=rospy.Subscriber("/joint_states",JointState,self.joint_callback)

        # do_action实例化后一直挂起，等待粗位置输入后执行
        self.all_infos_lock = threading.Lock() # 相机相关
        self.prim_thread = threading.Thread(target=self.do_action) # 执行动作planner
        self.prim_thread.start()         
        self.tool_pose_thread=threading.Thread(target=self.tool_pose_update) # 力、力矩信息相关
        self.tool_pose_thread.start()

    def force_callback(self,msg):

        """
        发布力和力矩
        """ 

        try:
            # 读取力和力矩信息
            x_force=msg.wrench.force.x
            y_force=msg.wrench.force.y
            z_force=msg.wrench.force.z
            x_torque=msg.wrench.torque.x
            y_torque=msg.wrench.torque.y
            z_torque=msg.wrench.torque.z
            # 将力和力矩分别存为一个数组            
            raw_force=np.array([x_force,y_force,z_force])
            raw_torque=np.array([x_torque,y_torque,z_torque])
            # 如果初始力和力矩为NOne，则将赋予读取到的值
            if (self.ori_force is None) and (self.ori_torque is None):
                self.ori_force = raw_force
                self.ori_torque = raw_torque
                print ('original wrench collected')
            # 如果初始力、力矩不为0，则用当前力和力矩减去初始（即使末端执行器没有接触螺钉，传感器的输出也有值，理解为未归零）
            elif self.collect == True:
                self.t_cur_force = raw_force-self.ori_force
                self.t_cur_torque = raw_torque-self.ori_torque
                self.cur_force = self.t_cur_force.tolist()
                self.cur_torque = self.t_cur_torque.tolist()
        except Exception as err:
            print("Exception happen in message call back:", err)
    
    def joint_callback(self,msg):

        """
        发布关节状态时的回调函数，将角度转为弧度并且转为列表后赋值给预先定义的变量
        """ 

        try:
            joint_state = np.array(msg.position) * 180 / np.pi
            self.joint_state = joint_state.tolist()
        except Exception as err:
            print("Exception happen in message call back:", err)

    def tool_pose_update(self):

        """
        一直运行,实现末端位姿、力/力矩实时更新
        """ 

        # 定义循环频率为10HZ
        rate = rospy.Rate(15)
        while not (rospy.is_shutdown() and self.prim_execution):
            try:
                # 由moveit获取末端执行器的位置，位姿信息
                pose = self.group.get_current_pose(self.effector).pose
                if (not pose is None) and self.collect:
                    # 末端位置、位姿6个值转换为列表
                    self.t_tool_pose = np.array([rospy.Time.now().to_sec(), 
                                                pose.position.x, 
                                                pose.position.y, 
                                                pose.position.z, 
                                                pose.orientation.x, 
                                                pose.orientation.y, 
                                                pose.orientation.z, 
                                                pose.orientation.w])
                    # 先将位置、位姿、力、力矩拼接为，后转换为1x14(8+3+3)
                    pose_wrench = np.concatenate((self.t_tool_pose,self.t_cur_force, self.t_cur_torque)).reshape([1,14])
                    # 将rospy.Time.now().to_sec()即该时刻存入列表
                    self.wrench_list.append(pose_wrench.tolist()[0])
            except Exception as e:
                print('Error:',e)

            rate.sleep()

    def start(self, pose):

        '''
        sleep表示暂时不能执行任何动作,没有粗位置
        'target_aim'= True为该项目第一个原语执行前提条件,满足则开始执行该项目任务
        '''

        # 初始化plc
        self.plc_control.set_return_zero()

        if self.action != 'sleep':
            print("Please start after previous task was done!")
            return False
        else:
            self.stage['target_aim']= True
            self.action = 'start'
            return True

    def auto_plan(self,original_stage):

        '''
        输入为初始的谓词状态,输出当前执行拆解的最短路径
        '''

        print ('start to plan')
        # original_stage一开始初始化时定义的状态
        ori_stage=original_stage

        # 建立动作原语集
        insert=PrimAction('insert')
        fumble=PrimAction('fumble')
        search=PrimAction('search_and_reinsert')
        disassemble=PrimAction('disassemble')
        prim_list=(insert,fumble,search,disassemble)

        # 基于FIFO的规划生成方法/判断任务是否完成，未完成的话再规划
        pathQueue=Queue(0)
        # 存入stage字典与一个空列表
        pathQueue.put([ori_stage,[]])

        # 循环执行直到PLAN结束
        plan_is_end=False
        while not plan_is_end:
            # 取出self.stage与空列表
            tmp_pair=pathQueue.get()
            tmp_stage=tmp_pair[0]
            tmp_path=tmp_pair[1]
            # 如已完成拆解则将stage再放入队列并且，定义plan为结束
            if tmp_stage['disassembled']==True:
                pathQueue.put(tmp_pair)
                plan_is_end=True
            else:
                # 用当前中台依次对比insert,fumble,search,disassemble的前提条件
                for primi in prim_list:
                    # 如果符合当前原语前提条件
                    if primi.able(tmp_stage)==True:
                        # 将当前原语的执行结果状态赋值（假装已完成当前原语）
                        new_stage=primi.action(tmp_stage)
                        new_path=[]
                        for n in tmp_path:
                            new_path.append(n)
                        # 将前原语插入列表即创建plan
                        new_path.append(primi.prim)
                        pathQueue.put([new_stage,new_path])
        path_list=[]
        while not pathQueue.empty():
            path=pathQueue.get()
            path_list.append(path[1])        

        #筛选出所有最短步数的规划方案
        min_step=100
        for path in path_list:
            if len(path)<min_step:
                min_step=len(path)
        path_list=[i for i in path_list if len(i)==min_step]
        print (path_list[0])
        return path_list[0]

    def do_action(self):

        """
        运行planner
        """ 
        
        # 实例化动作原语
        insert=PrimAction('insert')
        fumble=PrimAction('fumble')
        search=PrimAction('search_and_reinsert')
        disassemble=PrimAction('disassemble')
        prim_dict={
            'insert':insert,
            'fumble':fumble,
            'search_and_reinsert':search,
            'disassemble':disassemble}

        # 循环执行
        while self.prim_execution:
            if self.action== 'sleep':
                rospy.sleep(1)
                continue
            else:
                if self.action == 'start':
                    print('action==start do auto_plan')
                    # 规划执行顺序并且依次顺序执行
                    step_list = self.auto_plan(self.stage)
                    i = 0
                    self.action = step_list[i]
                    print(self.action)

                # 获取图片并判断结果
                if self.all_infos_lock.acquire():
                    infos = copy.deepcopy(self.all_infos)
                    self.all_infos.clear()
                    self.all_infos_lock.release()

                    if self.action in prim_dict.keys():
                        pre_is_ok = True
                        pic_state = infos['rgb_img']
                        size = 320
                        left = 1440
                        top = 560
                        pic_state = pic_state[top:top + size, left:left + size]        
                        # 保存数组图片
                        cv2.imwrite('np_check.png',pic_state)
                        # 谓词检查结果
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
                        # 遍历列表中所有原语前提条件       
                        for pre in (prim_dict[self.action]).pre:
                            # 如果当前状态下前提条件与原语定义的前提条件不相同则退出
                            if not self.stage[pre]==(prim_dict[self.action].pre)[pre]:
                                pre_is_ok=False
                                print("Precondition not satisfied")
                                break
                        if pre_is_ok==True:
                            # 这里的prims为调用单独原语文件的意思，action为原语定义中的原语名，即为前面提供索引
                            prim = self.prims[self.action]
                            # 将当前的实例赋值      
                            infos['planner_handler']=self
                            # 跳转具体原语的实现
                            self.ret_dict = prim.action(infos, self.ret_dict,self.plc_control)
                            print(self.action+"_is_finished")

                            # update effect/
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

    # 未调用/在do_action中重写了/用于获取图片
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

#################################################################################
# 两个都是相机相关信息
    def cam_info_cb(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_model_ready = True
        self.sub_camera_info.unregister()

    def callback(self, rgb_msg, depth_msg):

        '''
        相机相关回调函数，用于打包相机信息
        '''

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
###############################################################################

    # 此处未调用
    def __del__(self):
        self.prim_execution = False
        self.prim_thread.join()
    
    def print_pose(self,pose):

        '''
        输入位置和四元数表示的位姿，打印出位置和欧拉角
        '''

        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        # 由四元数转换欧拉角
        rpy = tf.transformations.euler_from_quaternion(q)
        print ('%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
            (self.effector, pose.position.x, pose.position.y, pose.position.z, \
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
            rpy[0], rpy[1], rpy[2]))

if __name__ == '__main__':

    try:
        # 初始化节点
        rospy.init_node('tstplanner-moveit', anonymous=True)
        # 启动planner
        planner = TSTPlanner('camera',
        '/camera/color/image_raw',
        '/camera/aligned_depth_to_color/image_raw',
        '/camera/color/camera_info')

        # 欧拉角转四元数，输入为（roll， pitch， yaw）    
        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, -0.5*math.pi)
        # 输入螺钉粗位置和位姿，位姿用四元数表示
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = 0.17
        pose_target.position.y = 0.7
        pose_target.position.z = 0.2
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        planner.start(pose_target)

        # 一直循环运行，直到节点关闭
        while not rospy.is_shutdown():
            # 代码运行到rospy.spin()之后会暂停会锁住这个程序/一般搭配回调函数使用
            rospy.spin()
        
        del planner
        
    except rospy.ROSInterruptException: # 节点中断异常时
        print("Shutting down")
        # 删除图像窗口
        cv2.destroyAllWindows()