import sys
import PySide2
from PySide2.QtWidgets import  QMessageBox,QFileDialog
from PySide2.QtUiTools import QUiLoader
from PyQt5.QtWidgets import QApplication
import time
import os
import numpy as np
import torch
from subprocess import Popen
import subprocess
import rospy
from std_msgs.msg import String
import sqlite3
from true_base import TrueBase
import moveit_commander
import tf
from geometry_msgs.msg import WrenchStamped, Pose
from PyQt5.QtGui import QFont
import signal

dirname = os.path.dirname(PySide2.__file__)
plugin_path = os.path.join(dirname, 'plugins', 'platforms')
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path
class SI:
    mainwin=None
    login=None

class login:

    def __init__(self):

        self.ui = QUiLoader().load('/home/ws/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/ur_control/scripts/true_sleeve_change/mainwindow.ui')
        self.ui.pushButton_start.clicked.connect(self.Launching_dependent_programs)
        self.ui.pushButton_start2.clicked.connect(self.Launching_mianprograms)
        self.ui.pushButton_getpose.clicked.connect(self.get_pose)
        self.ui.pushButton_inputpose.clicked.connect(self.inputpose)
        self.ui.pushButton_get_angle.clicked.connect(self.get_angle)
        self.ui.pushButton_end.clicked.connect(self.end)

        self.ui.show_pose1.setEnabled(False)
        self.ui.show_pose2.setEnabled(False)

        self.ui.show1.setEnabled(False)
        self.ui.show2.setEnabled(False)
        self.ui.show3.setEnabled(False)
        self.ui.show4.setEnabled(False)
        self.ui.show5.setEnabled(False)
        self.ui.show6.setEnabled(False)

        # self.ui.lineEdit.setEnabled(False)
    def Launching_dependent_programs(self):
        sh_file_path = '/home/ws/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/ur_control/scripts/true_sleeve_change/Launching_dependent_programs.sh'
        try :

            subprocess.run(["bash", sh_file_path], check=True)
        
        except subprocess.CalledProcessError:
            print("Error: Failed to run the .sh file")
        
    def Launching_mianprograms(self):
        sh_file_path = '/home/ws/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/ur_control/scripts/true_sleeve_change/Launching_mianprograms.sh'
        try :
            self.process = subprocess.run(["bash", sh_file_path], check=True)
        
        except subprocess.CalledProcessError:
            print("Error: Failed to run the .sh file")



    def end(self):
        # 结束进程
        process_ids = subprocess.getoutput("pgrep -f 'rosrun ur_control task1.py'").split('\n')
        for process_id in process_ids:
            try:
                os.kill(int(process_id), signal.SIGTERM)
            except ProcessLookupError:
                pass

        print("task1 terminated!")
# ----------------------------------------------------------------------------------------------------------------------------
    def get_pose(self):

        print('开始执行读数据库操作')
        conn = sqlite3.connect('your_database.db')
        cursor = conn.cursor()

        # 执行查询操作
        cursor.execute("SELECT * FROM robot_pose")

        # 获取所有行数据
        rows = cursor.fetchall()
        print('len(rows)',len(rows))
        print('rows',rows)

        # 关闭游标
        cursor.close()

        # 将查询结果转换成Pose对象的列表
        pose_list = []
        pose_list2 = []
        for row in rows:
            pose = Pose()
            pose.position.x = row[1]
            pose.position.y = row[2]
            pose.position.z = row[3]
            pose.orientation.x = row[4]
            pose.orientation.y = row[5]
            pose.orientation.z = row[6]
            pose.orientation.w = row[7]
            pose_list.append(pose.position)
            pose_list2.append(pose.orientation)
            print('pose_list',pose_list)
            # print('数据库读到的pose_list:', pose_list)
            # font = QFont()
            # font.setPointSize(8)  # 设置字体大小为12
            # self.ui.show_pose.setFont(font)
            self.ui.show_pose1.setText(str( pose_list))
            self.ui.show_pose2.setText(str( pose_list2))

        return pose_list


    def inputpose(self):
        # 从用户界面获取列表值
        input_data = self.get_input_data()
        print(input_data,"!!!!!!!!!!")

        # 将数据写入SQLite数据库
        self.write_to_database(input_data)

    def get_input_data(self):
        x = self.ui.input1.text().strip()
        y = self.ui.input2.text().strip()
        z = self.ui.input3.text().strip()
        row = self.ui.input4.text().strip()
        pitch = self.ui.input5.text().strip()
        yaw = self.ui.input6.text().strip()

        
        return [eval(x),eval(y),eval(z),eval(row),eval(pitch),eval(yaw)]

    def write_to_database(self, input_data):
        # 连接到SQLite数据库（如果不存在，它会创建一个）
        conn = sqlite3.connect('your_database.db')
        cursor = conn.cursor()

        # 创建表格（如果不存在），同时定义id字段为自增
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS your_table (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                value TEXT
            )
        ''')

        # 将数据插入表格
        for item in input_data:
            cursor.execute('INSERT INTO your_table (value) VALUES (?)', (item,))

        # 提交更改并关闭数据库连接
        conn.commit()
        conn.close()


    def get_angle(self):

        print('开始执行读数据库操作')
        conn = sqlite3.connect('your_database.db')
        cursor = conn.cursor()

        # 执行查询操作
        cursor.execute("SELECT * FROM robot_joint_angle")

        # 获取所有行数据
        rows = cursor.fetchall()
        print('len(rows)',len(rows))
        print('rows',rows)

        # 关闭游标
        cursor.close()

        # 将查询结果转换成Pose对象的列表

        for row in rows:
            self.ui.show1.setText(str(row[1]))
            self.ui.show2.setText(str(row[2]))
            self.ui.show3.setText(str(row[3]))
            self.ui.show4.setText(str(row[4]))
            self.ui.show5.setText(str(row[5]))
            self.ui.show6.setText(str(row[6]))

        return rows


    # def end():
    #     pass


if __name__=="__main__":
    app=QApplication(sys.argv)
    SI.loginwin = login()
    SI.loginwin.ui.show()
    sys.exit(app.exec())

