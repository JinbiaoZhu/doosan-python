import ast
import ctypes
import datetime
import logging
import os
import time
from typing import List

import matplotlib.pyplot as plt
import numpy as np


class Doosan:
    """
    斗山机械臂的python控制方案（Doosan-Python）：
        模式（目前）：终端模式、轨迹播放模式。
        形式（目前）：基于关节角、基于末端位姿。
        信息获取（目前）：实时关节角、实时末端位姿、实时关节力矩。
        信息记录（目前）：终端显示、日志保留。
        外设（目前）：夹抓。
    Doosan-Python Class主要实现的方法：
        前置下划线表示用户不可见；
        非前置下划线表示用户可见可修改。
            1. MoveJoint 终端输入方式实现基于关节角的机械臂运动；
            2. MoveLine 终端输入方式实现基于末端位姿的机械臂线性运动；
            3. ReplayTrajectory 离线脚本输入方式实现基于关节角或末端位姿的机械臂运动；

            4. ReadJoint 输出六个关节的当前角度值；
            5. ReadEndEffector 输出末端三维位置和姿态；
            6. ReadJointTorque 输出各个关节力矩；

            7. GripperControl 外设夹抓控制；

            8. _Wait 延时函数；
            9. _Legacy 跨语言的保留函数；
            10. Home 离线脚本输入方式实现基于关节角的回到起始位置；
            11. Initialize 初始化函数；
            12. QuitLoop 终端输入方式退出循环函数；
            13. Disconnect 退出连接函数；
            14. _Log 日志记录函数。

            15.__ctypes_encode 变量转换成 ctypes 数据类型
            16.__ctypes_decode 变量从 ctypes 数据类型转换成 python 数据类型
            17.__Variable_declaration 一些函数的输出类型声明

            18. Read_plot_torques 追踪力矩变化并记录和绘图
    """
    NUM_JOINTS = 6

    def __init__(self, address, libpath='./libdoosan.so', home=None):
        """
        初始化函数。
        :param address: 机械臂的IP地址
        :param libpath: 动态链接库的地址，默认位于当前py文件同级
        :param home: 基于关节角的机械臂初始位置
        """
        if home is None:
            home = [0, 0, 0, 0, 0, 0]
        assert isinstance(address, str), "The address must be a string!"
        self._address = bytes(address, 'utf-8')
        self._libpath = libpath
        self._home = home

        self._isInit = False
        self._isInitLog = False
        self._isLoop = None

        self._doosan = None

        self._Log('')

    def MoveJoint(self, jointsList: List, velocity: float, acceleration: float):
        """
        基于关节角的机械臂运动.
        :param jointsList:
        :param velocity:
        :param acceleration:
        :return:
        """
        jointsList_ = self.__ctypes_encode(jointsList)
        if isinstance(velocity, int): velocity = float(velocity)
        if isinstance(acceleration, int): acceleration = float(acceleration)
        velocity_ = self.__ctypes_encode(velocity)
        acceleration_ = self.__ctypes_encode(acceleration)
        flag = self._doosan.robotMoveJoint(jointsList_, velocity_, acceleration_)
        print(f"Move joint to {jointsList} successfully!")
        self._Log(f"Move joint {jointsList}, velocity: {velocity}, acceleration: {acceleration}.")
        return flag
        pass

    def MoveLine(self, poseture: List, velocity: List, acceleration: List):
        poseture_ = self.__ctypes_encode(poseture)
        velocity_ = self.__ctypes_encode(velocity)
        acceleration_ = self.__ctypes_encode(acceleration)
        flag = self._doosan.robotMoveEndEffector(poseture_, velocity_, acceleration_)
        self._Wait(5)
        print(f"Move line to {poseture} successfully!")
        self._Log(f"Move line {poseture}, velocity: {velocity}, acceleration: {acceleration}")
        return flag
        pass

    def ReplayTrajectory(self, filepath):

        with open(filepath, 'r', encoding='utf-8') as f:
            lineIndex = 0
            for line in f.readlines():
                lineIndex += 1
                command = line.strip()
                print(f"{command}, this is the {lineIndex}th line.")
                command = command.split(', ')
                mode = command[0]
                if mode not in ['j', 'l', 'op', 'of', 'q']:
                    print("You must write the 'j', 'l' and 'q' in first")
                    continue
                if mode == 'j':
                    self._Log("Read 'j'")
                    jointList = ast.literal_eval(command[1])
                    self._Log(f"Read joint angles {jointList}")
                    velocity = float(command[2])
                    acceleration = float(command[3])
                    self._Log(f"Read the velocity {velocity}, and acceleration {acceleration}")
                    self.MoveJoint(jointList, velocity, acceleration)
                if mode == 'l':
                    self._Log("Read 'l'")
                    poseture = ast.literal_eval(command[1])
                    self._Log(f"Read posetures {poseture}")
                    velocity = ast.literal_eval(command[2])
                    acceleration = ast.literal_eval(command[3])
                    self._Log(f"Read the velocity {velocity}, and acceleration {acceleration}")
                    self.MoveLine(poseture, velocity, acceleration)
                if mode == 'op':
                    self._Log("Read 'op'")
                    self.GripperControl(True)
                if mode == 'of':
                    self._Log("Read 'of'")
                    self.GripperControl(False)
                if mode == 'q':
                    self._Log("Read 'q', breaking...")
                    break
        pass

    def ReadJoint(self):
        temp = self._doosan.robotReadJoint()
        results = self.__ctypes_decode(temp)
        results = np.round(results, 2)
        self._Log(f"Get the result, ReadJoint, {results}")
        print(f"Read joint {results}")
        return results
        pass

    def ReadEndEffector(self):
        temp = self._doosan.robotReadEndEffector()
        results = self.__ctypes_decode(temp)
        results = np.round(results, 2)
        self._Log(f"Get the result, ReadEndEffector, {results}")
        print(f"Read end effector {results}")
        return results
        pass

    def ReadJointTorque(self):
        temp = self._doosan.robotReadJointTorque()
        results = self.__ctypes_decode(temp)
        results = np.round(results, 2)
        self._Log(f"Get the result, ReadJointTorque, {results}")
        print(f"Read joint torque {results}")
        return results
        pass

    def GripperControl(self, openoff: bool):
        assert isinstance(openoff, bool), "The variable input must be bool like!"
        self._doosan.robotGripperControl(ctypes.c_bool(openoff))
        self._Log(("Close " if openoff else "Open ") + "the gripper")
        print(("Close " if openoff else "Open ") + "the gripper")
        pass

    def _Wait(self, times: int):
        self._doosan.robotWait(ctypes.c_int(times))
        self._Log(f"Wait {times} seconds")
        print(f"Wait {times} seconds")
        pass

    def _Legacy(self):
        """
        此函数是跨平台使用，不写！
        :return:
        """
        pass

    def Home(self):
        goal = self._home
        goal_ = self.__ctypes_encode(self._home)
        velocity = 10.0
        velocity_ = self.__ctypes_encode(velocity)
        acceleration = 10.0
        acceleration_ = self.__ctypes_encode(acceleration)
        flag = self._doosan.robotMoveJoint(goal_, velocity_, acceleration_)
        self.GripperControl(True)
        print("Move home position successfully!")
        self._Log(f"Move home {goal}, velocity: {velocity}, acceleration: {acceleration}.")
        return flag
        pass

    def Initialize(self, mode: str, filepath=None):
        """
        载入动态链接库。
        通过IP地址连接机械臂。
        终端显示连接成功。
        将初始化信息写入日志。
        将机器人移动到初始位置中。
        判断运行模式。
        根据 mode 字符串进入不同模式。
        终端显示初始化成功。
        :return:
        """
        self._doosan = ctypes.CDLL(self._libpath)
        self._isInit = self._doosan.robotInitialize(self._address, len(self._address))
        print(f"Welcome to Doosan Robot!\nYour address is: {self._address}")
        self._isInit = True
        self._Log(f"{self._address}")
        self._Log(f"{self._libpath}")
        self._Log(f"{self._home}(joint angles).")

        self.Home()

        self.__Variable_declaration()

        assert mode in ["demo", "terminal", "offline", "user"], "The mode must be in 'demo', 'terminal', 'offline', " \
                                                                "'user'] "
        self._Log("Enter the " + mode + " mode")
        print("Initialize successfully!")
        if mode == "demo":
            self.Demo()
        elif mode == "terminal":
            self.Loop()
        elif mode == "offline":
            if filepath is None:
                raise "You dont enter the file path!"
            else:
                self.ReplayTrajectory(filepath=filepath)
        elif mode == "user":
            pass

    def QuitLoop(self):
        self._isLoop = self._doosan.robotQuitLoop()
        self._Log("User quit the loop")
        print("Quit the loop!")
        pass

    def Disconnect(self):
        self._Log("Disconnected.")
        self._doosan.robotDisconnect()
        pass

    def _Log(self, infos: str):
        if self._isInit is False and self._isInitLog is False:
            folder_name = 'log'
            path = os.path.join(os.getcwd(), folder_name)
            if not os.path.exists(path):
                os.makedirs(path)
            logging.basicConfig(filename=f'./{folder_name}/{datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}.log',
                                level=logging.DEBUG,
                                format="%(asctime)s - %(levelname)s - %(message)s")
        else:
            logging.info(infos)
        pass

    def __ctypes_encode(self, obj):
        if isinstance(obj, int):
            temp = ctypes.c_int(obj)
            self._Log(f"Change the {obj} to the ctypes {temp}")
        elif isinstance(obj, float):
            temp = ctypes.c_float(obj)
            self._Log(f"Change the {obj} to the ctypes {temp}")
        elif isinstance(obj, list):
            temp = (ctypes.c_float * len(obj))(*obj)
            self._Log(f"Change the {obj} to the ctypes {temp}")
        else:
            self._Log(f"Wrong in {obj} changing to the ctypes!")
            raise "The value(s) you input maybe wrong!"
        return temp

    def __ctypes_decode(self, res) -> np.array:
        """
        这个函数只在 ReadJoint、ReadEndEffector 和 ReadJointTorque 中才能被使用！
        :param res: 调用动态链接库获得的结果
        :return:
        """
        array_pointer = ctypes.cast(res, ctypes.POINTER(ctypes.c_float))
        self._Log("Generate the ctypes.POINTER of {res} to array pointer")
        array = np.ctypeslib.as_array(array_pointer, shape=(self.NUM_JOINTS,))
        self._Log("Generate the numpy's results array")
        return array
        pass

    def __Variable_declaration(self):
        self._doosan.robotReadJoint.restype = ctypes.POINTER(ctypes.c_float)
        self._Log("Set ReadJoint return type: ctypes.POINTER")
        self._doosan.robotReadJointTorque.restype = ctypes.POINTER(ctypes.c_float)
        self._Log("Set ReadJointTorque return type: ctypes.POINTER")
        self._doosan.robotReadEndEffector.restype = ctypes.POINTER(ctypes.c_float)
        self._Log("Set ReadEndEffector return type: ctypes.POINTER")
        print("Some methods have been reset return types successfully!")
        pass

    def Demo(self):
        for i in range(10):
            self.MoveJoint([0, 0, 90, 90, 0, 0], 10, 10)
            self._Wait(1)
            self.MoveJoint([0, 0, 90, 90, 90, 0], 10, 10)
            self._Wait(1)
            self.MoveJoint([0, 0, 90, 90, 0, 0], 10, 10)
            self._Wait(1)
            self.MoveJoint([0, 0, 0, 0, 0, 0], 10, 10)
            self._Wait(1)
        self._Log("Demo ends")
        print("Demo ends")
        pass

    def Loop(self):
        if self._isLoop is None:
            self._isLoop = True
        else:
            raise "The _isLoop has been set something before the loop!"

        print("This is the terminal mode.\nPlease enter the motion like:")
        while self._isLoop:
            print("'j' for joints, 'l' for lines, 'q' for quit, 'rj' for read joints, 'rp' for read positions, "
                  "'rt' for read torques, 'op' for gripper open, 'of' for gripper close.")
            print("/////////////////////////////////////////////////////////////////////////////////")
            motion = input("Enter your input char:")
            if motion not in ['j', 'l', 'q', 'rj', 'rp', 'rt', 'op', 'of']:
                print("You must enter the three 'j', 'l', 'q', 'rj', 'rp', 'rt', 'op' and 'of'.")
                continue
            if motion == 'j':
                self._Log("User enter 'j'")
                print("Please enter the joint angles:\nlike 0, 0, 0, 0, 0, 0")
                command = input("Enter your input values:").split(', ')
                temp = []
                for i in command:
                    temp.append(float(i))
                self._Log(f"User enter joint angles {temp}")
                self.MoveJoint(temp, 20, 20)
            if motion == 'l':
                self._Log("User enter 'l'")
                print("Please enter the postures:\nlike 594, 24, 571, 178, -90, -90")
                command = input("Enter your input values:").split(', ')
                temp = []
                for i in command:
                    temp.append(float(i))
                self._Log(f"User enter joint angles {temp}")
                self.MoveLine(temp, [20, 20], [20, 20])

            if motion == 'rj':
                self.ReadJoint()
            if motion == 'rp':
                self.ReadEndEffector()
            if motion == 'rt':
                self.ReadJointTorque()

            if motion == 'op':
                self.GripperControl(False)
            if motion == 'of':
                self.GripperControl(True)

            if motion == 'q':
                self.QuitLoop()
        pass

    def Read_plot_torques(self, num: int, ter: float, plot: bool):
        folder_name = 'record'
        _temp_list = []
        path = os.path.join(os.getcwd(), folder_name)
        if not os.path.exists(path):
            os.makedirs(path)
        fntxt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.txt'
        fnpng = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.png'
        fd = open(path + '/' + fntxt, 'w')

        index = 0
        self._Log("Start reading and ploting the torques...")
        while True:
            index += 1
            print(index)
            time.sleep(ter)
            res_list = self.ReadJointTorque().tolist()
            if plot:
                _temp_list.append(res_list)
            fd.write(str(res_list) + "\n")
            if index > num:
                break
        fd.close()
        self._Log("Reading and ploting the torques data saved")
        if plot:
            plt.figure()
            plt.plot(_temp_list)
            plt.legend(["1", "2", "3", "4", "5", "6"])
            plt.savefig(path + '/' + fnpng)
            self._Log("Reading and ploting the torques plot saved")
            plt.show()


