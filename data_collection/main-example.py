import PyTac3D
import time


# print('PyTac3D version is :', PyTac3D.PYTAC3D_VERSION)
SN = ''

frameIndex = -1 
sendTimestamp = 0.0
recvTimestamp = 0.0
P, N, D, F, Fr, Mr = None, None, None, None, None, None
a = 1
def Tac3DRecvCallback(frame, param):
    global SN, frameIndex, sendTimestamp, recvTimestamp, P, D, F, N
    
    print('')
    print('Tac3D1 sensor started')
    # 显示自定义参数
    # print the custom parameter
    print(param)

    # 获得传感器SN码，可用于区分触觉信息来源于哪个触觉传感器
    # get the SN code, which can be used to distinguish which Tac3D sensor the tactile information comes from
    SN = frame['SN']
    print(SN)
    # get the frame index
    frameIndex = frame['index']
    print(frameIndex)
    # get the timestamp
    sendTimestamp = frame['sendTimestamp']
    recvTimestamp = frame['recvTimestamp']
    P = frame.get('3D_Positions')
    N = frame.get('3D_Normals')
    D = frame.get('3D_Displacements')
    F = frame.get('3D_Forces')
    Fr = frame.get('3D_ResultantForce')
    Mr = frame.get('3D_ResultantMoment')
    if a == 0:
        print('111')

tac3d = PyTac3D.Sensor(recvCallback=Tac3DRecvCallback, port=9988, maxQSize=5, callbackParam = 'test param')

# 等待Tac3D传感器启动并传来数据
# Wait for the Tac3D sensor to start and send data
tac3d.waitForFrame(SN = 'DL1-GWM0001')
tac3d.waitForFrame(SN = 'DL1-GWM0002')
time.sleep(5)
tac3d.calibrate(SN = 'DL1-GWM0001')
tac3d.calibrate(SN = 'DL1-GWM0002')
time.sleep(5)

frame_1 = tac3d.getFrame()
a = 0


#5s
# time.sleep(5)

