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
import math
import threading
class TrueMove(TrueBase):
    def __init__(self, group_):
        super(TrueMove, self).__init__(group_)
        self.tf_lock = threading.Lock()
        self.stop = False
        self.fail = False
        self.data = []

    def get_contact_trajectory(self):
        radius = 0.005
        angle = 2.5        
        attempt = 100
        trajectory = []
        print('start to calculate trajectory')
        for i in range (attempt):
            target = geometry_msgs.msg.Pose()
            rad = 2 * math.pi * random.random()
            target.position.x = 0.2271 + radius * 0.5 * (random.random() + 1) * math.cos(rad)
            target.position.y = 0.9068 + radius * 0.5 * (random.random() + 1) * math.sin(rad)      
            target.position.z = 0.0725 - 0.25 * radius * random.random()
            ang_sum = angle * math.pi *random.random() / 180
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
            rospy.sleep(0.1)
            # self.tf_lock.release()
            if not target is None:
                trajectory.append(target)
                # print ("the %d-th trajectory"%(i))  
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

    def action(self, all_info, pre_result_dict,kalman,yolo):
        print("start to move")
        planner = all_info['planner_handler']
        contact_trajectory = self.get_contact_trajectory()
        target = geometry_msgs.msg.Pose()
        target.position.x = 0.2271
        target.position.y = 0.9068      
        target.position.z = 0.0725 + 0.01
        q = tf.transformations.quaternion_from_euler(-math.pi, 0, -0.5*math.pi)
        target.orientation.x = q[0]
        target.orientation.y = q[1]
        target.orientation.z = q[2]
        target.orientation.w = q[3]
        self.set_arm_pose(self.group, target, self.effector)
        rospy.sleep(0.2)
        planner.collect = True
        for pose in contact_trajectory:
            self.set_arm_pose(self.group, pose, self.effector)
            print ("fumble")
            rospy.sleep(0.1)
        planner.collect = False
        np_wrench_list = np.array(planner.wrench_list)
        np.savetxt("pose_wrench_"+str(rospy.Time.now().to_sec())+".csv",np_wrench_list, delimiter=",")
        print("Data is saved")
        rospy.is_shutdown(1)