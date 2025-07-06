#!/usr/bin/env python3
import os
import rospy
import argparse
import signal
import time
import pickle
import sys
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray


class Subscriber:
    def __init__(self, save_path):
        # 初始化ROS节点
        rospy.init_node('all_topics_subscriber', anonymous=True)
        
        self.stop = False
        self.save_path = save_path
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

        # 初始化变量
        self.last_print_time = rospy.Time.now()
        self.print_interval = rospy.Duration(0.1)  # 100ms打印间隔

        self.save_tactile_dict = {'tactile0': {'3Dshape': [], 'force': [], 'deform': [], 'timestamps': []},
                               'tactile1': {'3Dshape': [], 'force': [], 'deform': [], 'timestamps': []},
                               'start_time': None}
        self.save_gripper_data = {'gripper_pos': [], 'timestamps': []}
        

    def print_if_interval(self):
        """控制打印频率"""
        current_time = rospy.Time.now()
        if (current_time - self.last_print_time) > self.print_interval:
            self.last_print_time = current_time
            return True
        return False

    def tactile_0_force_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile0']['force'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_0/force: %.3f N" % msg.data)

    def tactile_1_force_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile1']['force'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_1/force: %.3f N" % msg.data)

    def tactile_0_array_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile0']['3Dshape'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_0/tactile_array: %s" % str([round(x, 3) for x in msg.data]))

    def tactile_1_array_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile1']['3Dshape'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_1/tactile_array: %s" % str([round(x, 3) for x in msg.data]))

    def deform_0_array_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile0']['deform'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_0/deform_array: %s" % str([round(x, 3) for x in msg.data]))

    def deform_1_array_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile1']['deform'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_1/deform_array: %s" % str([round(x, 3) for x in msg.data]))

    def timestamp_0_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile0']['timestamps'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_0/timestamp: %.3f" % msg.data)

    def timestamp_1_callback(self, msg):
        if msg.data is not None:
            self.save_tactile_dict['tactile1']['timestamps'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("tactile_1/timestamp: %.3f" % msg.data)

    def gripper_position_callback(self, msg):
        if msg.data is not None:
            self.save_gripper_data['gripper_pos'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("gripper_position: %.1f" % msg.data)

    def gripper_timestamp_callback(self, msg):
        if msg.data is not None:
            self.save_gripper_data['timestamps'].append(msg.data)
        # if self.print_if_interval():
        #     rospy.loginfo("gripper_position: %.1f" % msg.data)

    def run(self):
        print('start to record')
        rospy.Subscriber('tactile_0/force', Float32, self.tactile_0_force_callback)
        rospy.Subscriber('tactile_1/force', Float32, self.tactile_1_force_callback)
        rospy.Subscriber('tactile_0/tactile_array', Float32MultiArray, self.tactile_0_array_callback)
        rospy.Subscriber('tactile_1/tactile_array', Float32MultiArray, self.tactile_1_array_callback)
        rospy.Subscriber('tactile_0/deform_array', Float32MultiArray, self.deform_0_array_callback)
        rospy.Subscriber('tactile_1/deform_array', Float32MultiArray, self.deform_1_array_callback)
        rospy.Subscriber('tactile_0/timestamp', Float32, self.timestamp_0_callback)
        rospy.Subscriber('tactile_1/timestamp', Float32, self.timestamp_1_callback)
        rospy.Subscriber('gripper_position', Float32, self.gripper_position_callback)
        rospy.Subscriber('gripper_time', Float32, self.gripper_timestamp_callback)
        self.save_tactile_dict['start_time'] = int(time.time() * 1000)
        rospy.spin()
        
    
    def save_data(self):
        save_tactile_name = os.path.join(self.save_path, 'tactile')
        os.makedirs(save_tactile_name, exist_ok=True)
        with open(os.path.join(save_tactile_name, 'tactile.pkl'), 'wb') as f:
            pickle.dump(self.save_tactile_dict, f)
        print(f"save tactile data finished: {save_tactile_name}")
        
        save_gripper_name = os.path.join(self.save_path, 'gripper')
        os.makedirs(save_gripper_name, exist_ok=True)
        with open(os.path.join(save_gripper_name, 'gripper.pkl'), 'wb') as f:
            pickle.dump(self.save_gripper_data, f)
        print(f"save gripper data finished: {save_gripper_name}")


    def signal_handler(self, sig, frame):
        self.save_data()
        time.sleep(5)
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root_dir", 
        type=str, 
        default="/home/robotics/data_save", 
        help="Path to the base folder where data will be saved."
    )
    parser.add_argument(
        "--traj_number", 
        type=int, 
        default=255, 
        help="Trajectory ID."
    )
    args = parser.parse_args()

    save_path = os.path.join(args.root_dir, str(args.traj_number).zfill(4))
    os.makedirs(save_path, exist_ok=True)
    
    try:
        subscriber = Subscriber(save_path=save_path)
        subscriber.run()
    except rospy.ROSInterruptException:
        pass