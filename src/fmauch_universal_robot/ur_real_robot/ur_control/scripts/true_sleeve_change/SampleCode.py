#Sample Code

class Planner:
    def __init__(self, start_pose):
        self.tool_pose = None
        self.object_pose = None
        self.next_pose = None
        self.wrench = None
        self.start_pose = start_pose
        self.running = False

    def update_info(self):
        # 更新 tool_pose, object_pose, next_pose, wrench
        pass

    def start(self):
        self.running = True
        self.execute()

    def execute(self):
        while self.running:
            self.update_info()
            # 执行具体的机器人控制逻辑
            # ...

    def stop(self):
        self.running = False

import socket
import threading
import json
import time

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.planner = None
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        print("Server listening on port", port)

    def handle_client(self, client_socket):
        while True:
            message = client_socket.recv(1024).decode()
            if message:
                command = json.loads(message)
                if 'start_pose' in command:
                    self.planner = Planner(command['start_pose'])
                    self.planner.start()
                    self.send_planner_info(client_socket)
                elif 'stop' in command:
                    if self.planner:
                        self.planner.stop()
                # 其他指令处理

    def send_planner_info(self, client_socket):
        while self.planner and self.planner.running:
            info = {
                'tool_pose': self.planner.tool_pose,
                'object_pose': self.planner.object_pose,
                # 其他信息...
            }
            client_socket.send(json.dumps(info).encode())
            time.sleep(1)  # 每秒发送一次信息

    def start(self):
        while True:
            client_socket, _ = self.server_socket.accept()
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()



import socket
import json
import threading

class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))

    def send_start_command(self, start_pose):
        command = {'start_pose': start_pose}
        self.client_socket.send(json.dumps(command).encode())

    def send_stop_command(self):
        command = {'stop': True}
        self.client_socket.send(json.dumps(command).encode())

    def listen_for_planner_info(self):
        def listen():
            while True:
                info = self.client_socket.recv(1024).decode()
                print(json.loads(info))  # 打印接收到的信息

        listen_thread = threading.Thread(target=listen)
        listen_thread.start()

    def close_connection(self):
        self.client_socket.close()