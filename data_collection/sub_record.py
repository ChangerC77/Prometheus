#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float32MultiArray

class Subscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('all_topics_subscriber', anonymous=True)
        
        # 订阅所有相关话题
        rospy.Subscriber('tactile_0/force', Float32, self.tactile_0_force_callback)
        rospy.Subscriber('tactile_1/force', Float32, self.tactile_1_force_callback)
        rospy.Subscriber('tactile_0/tactile_array', Float32MultiArray, self.tactile_0_array_callback)
        rospy.Subscriber('tactile_1/tactile_array', Float32MultiArray, self.tactile_1_array_callback)
        rospy.Subscriber('tactile_0/deform_array', Float32MultiArray, self.deform_0_array_callback)
        rospy.Subscriber('tactile_1/deform_array', Float32MultiArray, self.deform_1_array_callback)
        rospy.Subscriber('tactile/start_time', Float32, self.start_time_callback)
        rospy.Subscriber('tactile_0/timestamp', Float32, self.timestamp_0_callback)
        rospy.Subscriber('tactile_1/timestamp', Float32, self.timestamp_1_callback)
        rospy.Subscriber('gripper_position', Float32, self.gripper_position_callback)
        
        # 初始化变量
        self.last_print_time = rospy.Time.now()
        self.print_interval = rospy.Duration(0.1)  # 100ms打印间隔
        
        rospy.loginfo("综合话题订阅器已启动，等待数据...")

    def print_if_interval(self):
        """控制打印频率"""
        current_time = rospy.Time.now()
        if (current_time - self.last_print_time) > self.print_interval:
            self.last_print_time = current_time
            return True
        return False

    def tactile_0_force_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_0/force: %.3f N" % msg.data)

    def tactile_1_force_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_1/force: %.3f N" % msg.data)

    def tactile_0_array_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_0/tactile_array: %s" % str([round(x, 3) for x in msg.data]))

    def tactile_1_array_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_1/tactile_array: %s" % str([round(x, 3) for x in msg.data]))

    def deform_0_array_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_0/deform_array: %s" % str([round(x, 3) for x in msg.data]))

    def deform_1_array_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_1/deform_array: %s" % str([round(x, 3) for x in msg.data]))

    def start_time_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile/start_time: %.3f" % msg.data)

    def timestamp_0_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_0/timestamp: %.3f" % msg.data)

    def timestamp_1_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("tactile_1/timestamp: %.3f" % msg.data)

    def gripper_position_callback(self, msg):
        if self.print_if_interval():
            rospy.loginfo("gripper_position: %.1f" % msg.data)

if __name__ == '__main__':
    try:
        subscriber = Subscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass