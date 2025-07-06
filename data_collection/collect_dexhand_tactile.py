# 触觉传感器的传回数据按照50Hz，左手传感器是"HDL1-0003"，右手传感器是"HDL1-0004".

import sys
import os
import argparse
from dexhand_client import DexHandClient
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '~/Tac-3D/DexHand-SDK-v1.1/pyDexHandClient/examples')))
import PyTac3D
import time
import pickle
from datetime import datetime

# 全局标志变量
grasp_executed = False

def get_timestamp():
    return str(int(time.time() * 1000 - 100))  # 100ms延迟


def save_to_pickle(data, base_folder, traj_number):
    """
    保存数据到指定的子文件夹中。
    Args:
        data (dict): 要保存的数据。
        base_folder (str): 存放所有实验数据的主文件夹路径。
        traj_number (str): 当前实验编号，用于创建子文件夹。
    """
    try:
        # 构造子文件夹路径
        sub_folder = os.path.join(base_folder, str(traj_number).zfill(4), 'tactile')
        os.makedirs(sub_folder, exist_ok=True)
        
        # 自动生成文件名
        file_name = f"{get_timestamp()}.pkl"
        file_path = os.path.join(sub_folder, file_name)

        # 保存数据到 pickle 文件
        with open(file_path, 'wb') as f:
            pickle.dump(data, f)

        # 返回子文件夹路径
        return sub_folder

    except Exception as e:
        print(f"Error saving data to {sub_folder}: {e}")

class Tacsensors:
    def __init__(self,  
                 left_tac3d_id="DL1-GWM0001", 
                 right_tac3d_id="DL1-GWM0002"):
        
        # 创建传感器数据存储实例
        self.Tac3D_name1 = left_tac3d_id
        self.Tac3D_name2 = right_tac3d_id
        self.tac3d = PyTac3D.Sensor(port=9988, maxQSize=5)
        # 等待 Tac3D-Desktop 端启动传感器并建立连接d
        self.tac3d.waitForFrame()
        time.sleep(5) # 5s
        self.tac3d.calibrate(self.Tac3D_name1)
        self.tac3d.calibrate(self.Tac3D_name2)

        self.grasp_executed = False
        self.first_frame1 = None
        self.first_frame2 = None

        print("finish init")


    def run(self):
        # 获取第一个传感器的当前帧，直到获得有效数据
        frame1 = None
        while frame1 is None or frame1['SN'] != self.Tac3D_name1:
            frame1 = self.tac3d.getFrame()

        # 获取第二个传感器的当前帧，直到获得有效数据
        frame2 = None
        while frame2 is None or frame2['SN'] != self.Tac3D_name2:
            frame2 = self.tac3d.getFrame()

        if self.first_frame1 is None:
            self.first_frame1 = frame1['3D_Positions']
        if self.first_frame2 is None:
            self.first_frame2 = frame2['3D_Positions']

        # 提取当前帧的 3D_Positions
        current_frame1 = frame1['3D_Positions']
        current_frame2 = frame2['3D_Positions']

        # 拼接数据成 (2, 2, 400, 3)
        frames = np.stack((
            np.array([self.first_frame1, current_frame1]),  # 左手
            np.array([self.first_frame2, current_frame2])   # 右手
        ))

        return {'tactile': frames}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default="~/umi/data_save")
    parser.add_argument("--traj_number", type=int, default=1)
    args = parser.parse_args()
    tac_sensor = Tacsensors(left_tac3d_id="DL1-GWM0001", right_tac3d_id="DL1-GWM0002")
    while True:
        tac_data = tac_sensor.run()
        save_to_pickle(tac_data, args.root_dir, args.traj_number)

