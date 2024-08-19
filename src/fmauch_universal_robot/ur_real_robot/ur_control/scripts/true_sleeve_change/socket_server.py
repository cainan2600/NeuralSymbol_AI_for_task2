import json
import socket
import threading
import time
# from true_task_1 import TSTPlanner
from test_task import TSTPlanner
import rospy
import math
import tf
import geometry_msgs
import cv2
from signal import signal, SIGPIPE, SIG_IGN


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.planner = None
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.server_socket.bind((self.host, self.port))
        self.clients = []
        "判断是否有客户端连接"
        self.is_client_connected = False
        # self.record = 0
    """监听连接的部分"""
    def start(self):
        self.server_socket.listen()
        print(f"服务器开始监听，地址为： {self.host}接口为: {self.port}")
        while True:
            client_socket, addr = self.server_socket.accept()
            print(f"接受来自地址为 {addr}的连接")
            self.clients.append(client_socket)
            self.is_client_connected = True
            threading.Thread(target=self.handle_client, args=(client_socket,)).start()

    """接受数据的部分"""
    def handle_client(self, client_socket):
        while True:
            try:
                # data = client_socket.recv(1024)
                data=json.loads(client_socket.recv(1024))
                # print(type(data.decode()))
                print(type(data))
                if not data:
                    break
                # print(f"Received data: {data.decode()}")
                print(f"Received data: {data}")
                # if "start" in data.decode():
                if "start" in data:    
                    # info=data.decode()
                    info=data
                    print()
                    start_pose=info["start"]
                    try:
                        # rospy.init_node('tstplanner-moveit', anonymous=True)
                        self.planner = TSTPlanner('camera', '/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', '/camera/color/camera_info')        
                        pose_target = geometry_msgs.msg.Pose()
                        # pose_target.position.x = start_pose[0]
                        # pose_target.position.y = start_pose[1]
                        # pose_target.position.z = start_pose[2]
                        # pose_target.orientation.x = start_pose[3]
                        # pose_target.orientation.y = start_pose[4]
                        # pose_target.orientation.z = start_pose[5]
                        # pose_target.orientation.w = start_pose[6]
                        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, -0.5*math.pi)
                        pose_target.position.x = 0.1
                        pose_target.position.y = 0.605
                        pose_target.position.z = 0.1
                        pose_target.orientation.x = quat[0]
                        pose_target.orientation.y = quat[1]
                        pose_target.orientation.z = quat[2]
                        pose_target.orientation.w = quat[3]
                        self.planner.start(pose_target)
                        self.start_input_thread()  # Start listening for input
                        while not rospy.is_shutdown():
                            rospy.spin()
                    except rospy.ROSInterruptException:
                        print("Shutting down")
                        cv2.destroyAllWindows()
                # elif "shutdown" in data.decode():
                elif "shutdown" in data:
                    if self.planner:
                        del self.planner
            except ConnectionResetError:
                break
        client_socket.close()


    """发送数据部分"""
    def send_message_to_all_clients(self, message):
        # for client in self.clients:
        client=self.clients[0]
        client.sendall(bytes(message.encode("utf-8")))
                # client.sendall(message.encode())
                


    def start_input_thread(self):
        def handle_input():
            # 等待直到有客户端连接
            while not self.is_client_connected:
                time.sleep(1)  # 短暂等待，避免密集循环
            #下面为发送数据的部分，此处测试为发送6条
            # for u in range(6):
            #     """只有有客户端连接的时候才发送数据"""
            #     if self.is_client_connected:
            #             # message={"Mechanical arm":[0.1,0.2,0.3,0.4,0.5,0.6,23],"Current screw":[1,2,3,4,5,6,7],"Sleeve type":[2]}

            #             message = {"Preparation screws": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 23],
            #                        "Sleeve type": [1], "Screw type": [2],"Motor speed":[3],"Power":[1,2,3],"Moment":[3,4,4]}
            #             message=json.dumps(message)
            #             self.send_message_to_all_clients(message+"\n")
                        # time.sleep(5)
            while not self.planner is None:
                if self.is_client_connected and self.planner.prim_execution:
                # if self.server_socket.accept():
                    msg={}
                    if not self.planner.tool_pose is None:
                        msg["Mechanical arm"]= self.planner.tool_pose
                    if not self.planner.curr_ob_pose is None:
                        msg["Current screw"]= self.planner.curr_ob_pose
                    if not self.planner.next_ob_pose is None:
                        msg["Preparation screws"]= self.planner.next_ob_pose
                    if not self.planner.cur_force is None:
                        msg["Power"]= self.planner.cur_force
                    if not self.planner.cur_torque is None:
                        msg["Moment"]= self.planner.cur_torque
                    # if not self.planner.sleeve_type is None:
                    #     msg["Sleeve type"]= self.planner.sleeve_type
                    # if not self.planner.aim_bolt_type is None:
                    #     msg["Screw type"]= self.planner.aim_bolt_type
                    # if not self.planner.motor_speed is None:
                    #     msg["Motor speed"]= self.planner.motor_speed
                    if not self.planner.joint_state is None:
                        print('have angle state')
                        msg["Angle"]= self.planner.joint_state
# have_coarse_pose':False, 'above_bolt':False,'target_aim':False, 'target_clear':True,'above_center_c':False,'target_match':False,'cramped':False,'disassembled':False
                    if not self.planner.stage is None:
                        if self.planner.stage['have_coarse_pose'] == True:
                            msg["State_now?"]= [0.0]
                            if self.planner.stage['above_bolt'] == True:
                                msg["State_now?"]= [1.0]
                                if self.planner.stage['target_aim'] == True:
                                    msg["State_now?"]= [2.0]
                                    if self.planner.stage['disassembled'] == True:
                                        msg["State_now?"]= [3.0]
                    # msg = {
                    #     "Mechanical arm": self.planner.tool_pose,
                    #     "Current screw": self.planner.curr_ob_pose,
                    #     "Preparation screws": self.planner.next_ob_pose,
                    #     "Power": self.planner.cur_force,
                    #     "Moment": self.planner.cur_torque,
                    #     "Sleeve type": self.planner.sleeve_type,
                    #     "Screw type": self.planner.aim_bolt_type,
                    #     "Motor speed": self.planner.motor_speed
                    # }
                    if msg:
                        # for i in msg.keys():
                        #     print(i)   #输出键
                        #     print(msg[i])  #输出值
                        message=json.dumps(msg)
                        json_len=len(message)
                        print(json_len)
                        self.send_message_to_all_clients(message+"\n")
                        # self.send_message_to_all_clients(message)
                        # self.record += 1
                        print('message is sent')
                    time.sleep(0.1)  # 每秒发送10次信息            
        input_thread = threading.Thread(target=handle_input)
        # input_thread.daemon = True
        input_thread.start()
    def close(self):
        self.server_socket.close()

if __name__ == "__main__":
    # rospy.init_node('tstplanner-moveit', anonymous=True)
    signal(SIGPIPE,SIG_IGN)
    rospy.init_node('tstplanner-moveit')    
    server = Server('192.168.56.100',2323)
    server_thread = threading.Thread(target=server.start)
    server_thread.start()