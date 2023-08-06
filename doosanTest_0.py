import ctypes

import numpy as np

doosan = ctypes.CDLL("./libdoosan.so")

is_Loop, is_init = False, 0

while is_init != 1:
    address = b"192.168.5.100"
    length = len(address)
    is_init = doosan.robotInitialize(address, length)

doosan.robotHome()

is_Loop = True
while is_Loop:
    # goal1 = [0, 0, 90, 90, 0, 0]
    # goal1 = (ctypes.c_float * len(goal1))(*goal1)

    # goal2 = [0, 0, 0, 0, 0, 0]
    # goal2 = (ctypes.c_float * len(goal2))(*goal2)

    # goal3 = [594, 24, 571, 178, -90, -90]
    # goal3 = (ctypes.c_float * len(goal3))(*goal3)

    # velocity = ctypes.c_float(10)
    # velocities = [10, 10]
    # velocities = (ctypes.c_float * len(velocities))(*velocities)
    # acceleration = ctypes.c_float(10)
    # accelerations = [10, 10]
    # accelerations = (ctypes.c_float * len(accelerations))(*accelerations)

    # waitflag = 0

    # res = doosan.robotMoveJoint(goal1, velocity, acceleration)

    doosan.robotReadJoint.restype = ctypes.POINTER(ctypes.c_float)
    res = doosan.robotReadJoint()
    array_pointer = ctypes.cast(res, ctypes.POINTER(ctypes.c_float))
    array_size = 6  # 数组大小
    my_array = np.ctypeslib.as_array(array_pointer, shape=(array_size,))
    my_array = np.round(my_array, 2)
    print(my_array)

    # doosan.robotReadJointTorque.restype = ctypes.POINTER(ctypes.c_float)
    # res = doosan.robotReadJointTorque()
    # array_pointer = ctypes.cast(res, ctypes.POINTER(ctypes.c_float))
    # array_size = 6  # 数组大小
    # my_array_ = np.ctypeslib.as_array(array_pointer, shape=(array_size,))
    # print(my_array_)

    # res = doosan.robotMoveEndEffector(goal3, velocities, accelerations)

    # res = doosan.robotGripperControl(ctypes.c_bool(True))
    # res = doosan.robotGripperControl(ctypes.c_bool(False))

    # c = input("输入字母‘c’以继续；输入字母‘q’以结束。")
    # if c == 'q':
    #     is_Loop = doosan.robotQuitLoop()

doosan.robotDisconnect()
