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
    def __init__(self, group_):
        super(TrueSearchAndReinsert, self).__init__(group_)
        self.tf_lock = threading.Lock()
        self.model_predictor = MyModelPredictor('lstm_model_0221.pth', 'data_mean_0221.npy', 'data_std_0221.npy')
        self.stop = False
        self.fail = False
        self.data = []

    def get_search_trajectory(self, attempt, radius, rotation=0):
        trajectory = []
        delta_angle = 30
        a = 0.01
        b = 0.02
        radius = radius
        scale_angle = delta_angle * math.pi / 180
        
        print("get_search_trajectory")

        for i in range(int(attempt)):
            a = (i * 0.01) + 0.01
            b = (i * 0.02) + 0.02
            for j in range(int(360 / delta_angle)):
                temp_angle = scale_angle * j
                tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
                tgt_pose_in_effector_frame.position.x = a * math.cos(temp_angle)
                tgt_pose_in_effector_frame.position.y = b * math.sin(temp_angle)
                tgt_pose_in_effector_frame.position.z = 0
                q = tf.transformations.quaternion_from_euler(0, 0, 0)
                tgt_pose_in_effector_frame.orientation.x = q[0]
                tgt_pose_in_effector_frame.orientation.y = q[1]
                tgt_pose_in_effector_frame.orientation.z = q[2]
                tgt_pose_in_effector_frame.orientation.w = q[3]
                tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                                    "base_link",
                                                                    tgt_pose_in_effector_frame,
                                                                    rospy.Time.now())
                if not tgt_pose_in_world_frame is None:
                    trajectory.append(tgt_pose_in_world_frame)
        if len(trajectory) > 0:
            print("search trajectory collected") 
        return trajectory



    def get_reinsert_trajectory(self, vector):

        print('get_recramp_trajectory')

        scale_step=0.0105
        trajectory = []
        start_pose= self.group.get_current_pose(self.effector).pose
        tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_effector_frame.position.x = 0
        tgt_pose_in_effector_frame.position.y = 0
        tgt_pose_in_effector_frame.position.z = -0.01
        q= tf.transformations.quaternion_from_euler(0, 0, 0)
        tgt_pose_in_effector_frame.orientation.x = q[0]
        tgt_pose_in_effector_frame.orientation.y = q[1]
        tgt_pose_in_effector_frame.orientation.z = q[2]
        tgt_pose_in_effector_frame.orientation.w = q[3]
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                            "base_link",
                                                            tgt_pose_in_effector_frame,
                                                            rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)

        tgt_pose_in_effector_frame.position.x = vector[0] * scale_step
        tgt_pose_in_effector_frame.position.y = vector[1] * scale_step
        tgt_pose_in_effector_frame.position.z = 0
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                        "base_link",
                                                        tgt_pose_in_effector_frame,
                                                        rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)
        if len(trajectory) > 0:
            print ("reinsert trajectoryl collected")
        return trajectory
        



    def action(self, all_info, pre_result_dict,plc):

        planner = all_info['planner_handler']     
        planner.collect = True

        # ee_pose = self.group.get_current_pose(self.effector).pose
        # research_trajectory = self.get_search_trajectory(attempt=1, radius=0)
        research_trajectory = self.get_search_trajectory(attempt=5, radius=0)
        print("ready to reaserch")
        for point1 in research_trajectory:

            self.set_arm_pose(self.group, point1, self.effector)
            print("research")

            vector_x = - planner.cur_force[0]
            vector_y = - planner.cur_force[1]
            vector_xy = math.sqrt(pow(vector_x, 2) + pow(vector_y, 2))
            if vector_xy > 2:
                vector = [vector_x / vector_xy, vector_y / vector_xy]

                reinsert_trajectory = self.get_reinsert_trajectory(vector=vector)
                print("ready to reinsert")
                for point2 in reinsert_trajectory:
                    self.set_arm_pose(self.group, point2, self.effector)
                    print("reinsert")
                print('end')
                break
        print("end")
        rospy.sleep(0.2)
        return {'success': True}    

