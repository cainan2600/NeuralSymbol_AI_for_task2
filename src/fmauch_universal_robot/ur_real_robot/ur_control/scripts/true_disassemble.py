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
class TrueDisassemble(TrueBase):
    def __init__(self, group_):
        super(TrueDisassemble, self).__init__(group_)
        self.tf_lock = threading.Lock()
        self.stop = False
        self.fail = False
        self.data = []

    def action(self, all_info, pre_result_dict,plc):
        # 可以访问planner中定义的实例
        planner = all_info['planner_handler']
        # 设置末端执行器转速并进行拆解     
        plc.set_effector_star_neg(3000)
        planner.motor_speed=plc.read_effector_speed()
        rospy.sleep(0.5)
        planner.motor_speed=plc.read_effector_speed()
        rospy.sleep(0.5)
        planner.motor_speed=plc.read_effector_speed()
        rospy.sleep(0.5)
        planner.motor_speed=plc.read_effector_speed()
        print("speed:",plc.read_effector_speed())
        # 一直读取力矩信息，当小于0.25时，拆解成功，停止旋转
        while True:
            planner.motor_speed=plc.read_effector_speed()
            if planner.cur_torque[2]<0.25:
                break
        planner.collect = False
        rospy.sleep(2)
        plc.set_effector_stop()
        # 获取当前位姿
        next_pose = self.group.get_current_pose(self.effector).pose
        # 因拆解完成，上台末端执行器
        next_pose.position.z += 0.05
        self.set_arm_pose(self.group, next_pose, self.effector)
        rospy.sleep(0.2)
        return {'success': True}