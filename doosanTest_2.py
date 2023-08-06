from DoosanV1 import Doosan

doosan = Doosan("192.168.5.100")
doosan.Initialize(mode='user')

doosan.MoveJoint([0, 0, 90, 0, 90, 0], 25, 25)

doosan.Read_plot_torques(100, 0.2, True)

doosan.Disconnect()
