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

def write_to_database(input_data):
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
        
def read():
        conn = sqlite3.connect('your_database.db')
        cursor = conn.cursor()

                # 执行查询以检索最后一次插入的数据
        cursor.execute('SELECT * FROM your_table ORDER BY id DESC LIMIT 6')
        data = cursor.fetchmany(6)
        final_data  = []
        for i in range(6):
            final_data.append(eval(data[5-i][1]))
                    # final_data形式：[111, 222, 333, 444, 555, 666] [x,y,z,r,p,y]
                # 关闭数据库连接
        conn.close()
        print("!!") 
        print(final_data) 

write_to_database([77777777,2,3,4,5,33.333])
read()