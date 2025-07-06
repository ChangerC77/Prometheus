#!/usr/bin/env python
import PyTac3D
import time
import os
import subprocess
import argparse
import pickle
import signal
import sys
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

class TacSensor:
    def __init__(self, SNs, port, save_path):
        self.SN1, self.SN2 = SNs
        self.port = port
        self.P, self.N, self.D, self.F, self.Fr, self.Mr = None, None, None, None, None, None

        # 初始化 ROS 节点
        rospy.init_node('tac_sensor_node', anonymous=True)
        self.pub_force_SN1 = rospy.Publisher('/tac_sensor/force_SN1', WrenchStamped, queue_size=10)
        self.pub_force_SN2 = rospy.Publisher('/tac_sensor/force_SN2', WrenchStamped, queue_size=10)

        tac3d_core_path = os.path.expanduser('~/Tac3D-SDK-v3.3.0/Tac3D-Core/linux-x86_64')
        tac_core = os.path.join(tac3d_core_path, 'Tac3D')
        config_path = os.path.join(tac3d_core_path, 'config')

        # 启动传感器1和传感器2
        start_sensor_1_cmd = f"{tac_core} -c {config_path}/{self.SN1} -i 127.0.0.1 -p {port}"
        start_sensor_2_cmd = f"{tac_core} -c {config_path}/{self.SN2} -i 127.0.0.1 -p {port}"
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_1_cmd])
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_2_cmd])

        self.save_data_dict = {
            self.SN1: {'tactile': [], 'deform': [], 'force': [], 'timestamps': []},
            self.SN2: {'tactile': [], 'deform': [], 'force': [], 'timestamps': []},
            'start_time': None
        }
        
        self.first_frame = True  # 修正拼写错误
        self.stop = False
        self.save_path = save_path

    def ensure_float(self, value):
        """确保值被转换为float类型"""
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def Tac3DRecvCallback(self, frame, param):
        SN = frame['SN']

        if self.first_frame:
            self.start_time = int(time.time() * 1000)
            self.first_frame = False
            self.save_data_dict['start_time'] = self.start_time
        
        timestamp = frame['sendTimestamp'] * 1000.0
        self.P = frame.get('3D_Positions')
        self.D = frame.get('3D_Displacements')
        self.F = frame.get('3D_Forces')

        # 发布 ROS topic
        if self.F is not None and len(self.F) >= 3:  # 确保有足够的数据
            try:
                force_msg = WrenchStamped()
                force_msg.header.stamp = rospy.Time.now()
                force_msg.header.frame_id = SN
                
                # 确保转换为float类型
                force_msg.wrench.force.x = self.ensure_float(self.F[0])
                force_msg.wrench.force.y = self.ensure_float(self.F[1])
                force_msg.wrench.force.z = self.ensure_float(self.F[2])
                
                # 扭矩值设为0（如果不使用）
                force_msg.wrench.torque.x = 0.0
                force_msg.wrench.torque.y = 0.0
                force_msg.wrench.torque.z = 0.0

                if SN == self.SN1:
                    self.pub_force_SN1.publish(force_msg)
                elif SN == self.SN2:
                    self.pub_force_SN2.publish(force_msg)
            except Exception as e:
                rospy.logerr(f"Error publishing force data for {SN}: {str(e)}")

        # 保存原始数据
        try:
            if SN == self.SN1:
                self.save_data_dict[self.SN1]['tactile'].append(self.P)
                self.save_data_dict[self.SN1]['timestamps'].append(timestamp)
                self.save_data_dict[self.SN1]['deform'].append(self.D)
                self.save_data_dict[self.SN1]['force'].append(self.F)
            elif SN == self.SN2:
                self.save_data_dict[self.SN2]['tactile'].append(self.P)
                self.save_data_dict[self.SN2]['timestamps'].append(timestamp)
                self.save_data_dict[self.SN2]['deform'].append(self.D)
                self.save_data_dict[self.SN2]['force'].append(self.F)
        except Exception as e:
            rospy.logerr(f"Error saving data for {SN}: {str(e)}")

    def save_data(self, save_data_dict, ftype):
        try:
            if ftype == 'pickle':
                with open(os.path.join(self.save_path, 'tactile.pkl'), 'wb') as f:
                    pickle.dump(save_data_dict, f)
                rospy.loginfo("Tactile data saved successfully.")
        except Exception as e:
            rospy.logerr(f"Error saving data: {str(e)}")
    
    def record_data(self):
        try:
            self.tac3d = PyTac3D.Sensor(
                recvCallback=self.Tac3DRecvCallback, 
                port=self.port, 
                maxQSize=10, 
                callbackParam='tactile_sensor'
            )
            self.tac3d.waitForFrame(SN=self.SN1)
            self.tac3d.waitForFrame(SN=self.SN2)
            time.sleep(2)
            self.tac3d.calibrate(SN=self.SN1)
            self.tac3d.calibrate(SN=self.SN2)
            time.sleep(2)

            rospy.loginfo("Start recording tactile data...")
            
            while not rospy.is_shutdown() and not self.stop:
                time.sleep(0.1)
                
        except Exception as e:
            rospy.logerr(f"Error in record_data: {str(e)}")
        finally:
            if hasattr(self, 'tac3d'):
                self.tac3d.disconnect()

    def signal_handler(self, sig, frame):
        self.stop = True
        self.save_data(self.save_data_dict, ftype='pickle')
        rospy.signal_shutdown("Received shutdown signal")
        sys.exit(0)

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--root_dir", type=str, default="/home/robotics/umi/data_save")
        parser.add_argument("--traj_number", type=int, default=255)
        args = parser.parse_args()

        save_path = os.path.join(os.path.expanduser(args.root_dir), str(args.traj_number).zfill(4), 'tactile')
        os.makedirs(save_path, exist_ok=True)
        
        tacsensor = TacSensor(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988, save_path=save_path)
        signal.signal(signal.SIGTERM, tacsensor.signal_handler)
        signal.signal(signal.SIGINT, tacsensor.signal_handler)  # 添加Ctrl+C处理
        tacsensor.record_data()
    except Exception as e:
        rospy.logerr(f"Main execution error: {str(e)}")
        sys.exit(1)