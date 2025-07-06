import pyrealsense2 as rs
import numpy as np
import os
import pickle
import time
import argparse
import threading
import sys
import select


class Realsense:
    def __init__(self, root_dir, traj_number=1):
        self.root_dir = root_dir
        self.traj_save = os.path.join(root_dir, str(traj_number).zfill(4), 'camera')
        os.makedirs(self.traj_save, exist_ok=True)

        self.pipeline_list = list()
        self.config_list = list()
        self.connect_device = list()
        self.devices_name = list()

        for device in rs.context().devices:
            device_name = device.get_info(rs.camera_info.name)
            if device_name.lower() != 'platform camera':
                self.connect_device.append(device.get_info(rs.camera_info.serial_number))
                self.devices_name.append(device_name)

        for i in range(len(self.connect_device)):
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.connect_device[i])
            if 'L515' in self.devices_name[i]:
                config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            elif 'D405' in self.devices_name[i]:
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            elif 'D435' in self.devices_name[i]:
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            self.pipeline_list.append(pipeline)
            self.config_list.append(config)

            pipeline.start(config)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.running = True

    def stop_on_keypress(self):
        """监听标准输入，当用户输入 's' 并按下回车时停止运行"""
        print("Press 's' and Enter to stop...")
        while self.running:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip()
                if user_input.lower() == 's':
                    print("Stopping RealSense...")
                    self.running = False  # 设置为 False，通知主线程停止
                    break

    def save_data(self, depth_image, color_image, timestamp, save_dir):
        # Prepare data dictionary
        data = {
            "depth_image": depth_image,
            "color_image": color_image,
        }
        
        file_name = f"{timestamp}.pkl"
        file_path = os.path.join(save_dir, file_name)
        
        with open(file_path, "wb") as f:
            pickle.dump(data, f)


    def run(self):
        # stop_thread = threading.Thread(target=self.stop_on_keypress)
        # stop_thread.daemon = True
        # stop_thread.start()
        try:
            while self.running:
                for i, pipeline in enumerate(self.pipeline_list):
                    frames = pipeline.wait_for_frames()
                    aligned_frames = self.align.process(frames)
                    aligned_depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()

                    if not aligned_depth_frame or not color_frame:
                        continue
                    timestamp = str(int(round(time.time(),3) * 1000))  # 颜色帧时间戳
                    depth_image = np.asanyarray(aligned_depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())

                    save_camera_name = self.devices_name[i] + '_' + self.connect_device[i]
                    save_path = os.path.join(self.traj_save, save_camera_name.replace(' ', '_'))
                    os.makedirs(save_path, exist_ok=True)
                    self.save_data(depth_image, color_image, timestamp, save_path)

                time.sleep(0.05)

        finally:
            self.running = False
            for pipeline in self.pipeline_list:
                pipeline.stop()
            print("All pipelines stopped.")

def main():
    parser = argparse.ArgumentParser(description="Run RealSense data collection.")
    parser.add_argument(
        "--root_dir", 
        type=str, 
        default="/home/robotics/data_save", 
        help="Path to the base folder where data will be saved."
    )
    parser.add_argument(
        "--traj_number", 
        type=int, 
        default=1, 
        help="Trajectory ID."
    )
    args = parser.parse_args()

    realsense = Realsense(root_dir=args.root_dir, traj_number=args.traj_number)
    realsense.run()

if __name__ == "__main__":
    main()