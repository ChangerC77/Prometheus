import PyTac3D
import time
import os
import subprocess
import argparse
import pickle
import signal
import sys


class TacSensor:

    def __init__(self, SNs, port):
        self.SN1, self.SN2 = SNs
        self.port = port
        self.P, self.N, self.D, self.F, self.Fr, self.Mr = None, None, None, None, None, None

        # tac3d_core_path = '/home/robotics/Tac3D-SDK-v3.3.0/Tac3D-Core/linux-x86_64'
        # tac_core = os.path.join(tac3d_core_path, 'Tac3D')
        # config_path = os.path.join(tac3d_core_path, 'config')
        # start_sensor_1_cmd = f"{tac_core} -c {config_path}/{self.SN1} -i 127.0.0.1 -p {port}"
        # start_sensor_2_cmd = f"{tac_core} -c {config_path}/{self.SN2} -i 127.0.0.1 -p {port}"
        # subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_1_cmd])
        # subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_2_cmd])

        self.save_data_dict = {self.SN1: {'tactile': [], 'deform': [], 'force': [], 'timestamps': []},
                               self.SN2: {'tactile': [], 'deform': [], 'force': [], 'timestamps': []},
                               'start_time': None}
        
        self.stop = False
        

    def Tac3DRecvCallback(self, frame, param):
        SN = frame['SN']
        
        timestamp = frame['sendTimestamp'] * 1000.0
        self.P = frame.get('3D_Positions')
        self.D = frame.get('3D_Displacements')
        self.F = frame.get('3D_Forces')
        # self.N = frame.get('3D_Normals')
        # self.Fr = frame.get('3D_ResultantForce')
        # self.Mr = frame.get('3D_ResultantMoment')

        if SN == self.SN1:
            self.save_data_dict[self.SN1]['tactile'].append(self.P) # (400, 3) float64
            self.save_data_dict[self.SN1]['timestamps'].append(timestamp) # float32
            self.save_data_dict[self.SN1]['deform'].append(self.D) # (400, 3) float64
            self.save_data_dict[self.SN1]['force'].append(self.F) # (400, 3) float64
        elif SN == self.SN2:
            self.save_data_dict[self.SN2]['tactile'].append(self.P)
            self.save_data_dict[self.SN2]['timestamps'].append(timestamp)
            self.save_data_dict[self.SN2]['deform'].append(self.D)
            self.save_data_dict[self.SN2]['force'].append(self.F)

        
    def save_data(self, save_data_dict, ftype):
        if ftype == 'pickle':
            with open(os.path.join(self.save_path, 'tactile.pkl'), 'wb') as f:
                pickle.dump(save_data_dict, f)
        
        time.sleep(5)

        print("Tactile data saved.")
    
    def record_data(self):
        self.tac3d = PyTac3D.Sensor(recvCallback=self.Tac3DRecvCallback, port=self.port, maxQSize=10, callbackParam='test param')
        self.tac3d.waitForFrame(SN=self.SN1)
        self.tac3d.waitForFrame(SN=self.SN2)
        time.sleep(2)
        self.tac3d.calibrate(SN=self.SN1)
        self.tac3d.calibrate(SN=self.SN2)
        time.sleep(2)
        
        while True:
            if self.stop:
                break
            
    def signal_handler(self, sig, frame):
        if sig == signal.SIGTERM:
            self.stop = True
        
        sys.exit(0)

if __name__ == '__main__':
    tacsensor = TacSensor(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988)
    signal.signal(signal.SIGTERM, tacsensor.signal_handler)
    tacsensor.record_data()

    
    

