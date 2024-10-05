#!/usr/bin/python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, square

class ReferencePathCreator:
    def __init__(self):
        rospy.init_node('reference_path_creator')
        self.pub_path = rospy.Publisher('/reference_path', Path, queue_size=1)
        self.hz = rospy.get_param('/reference_path_creator/publish_frequency', 10.0)
        self.wave_hz = rospy.get_param('/reference_path_creator/wave_hz', 1.0)
        self.fs = rospy.get_param('/reference_path_creator/fs', 1000.0)
        self.cutoff = rospy.get_param('/reference_path_creator/cutoff', 25.0)
        self.order = rospy.get_param('/reference_path_creator/filter_order', 6)
        self.len = rospy.get_param('/reference_path_creator/len', 10.0) 

    # ローパスフィルタを設計する関数
    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs  # ナイキスト周波数
        normal_cutoff = cutoff / nyq  # 正規化カットオフ周波数
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a
    
    # ローパスフィルタを適用する関数
    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order)
        y = lfilter(b, a, data)
        return y
    
    def timer_callback(self, _):
        t = np.linspace(0, self.len, int(self.len * self.fs), endpoint=False)
        square_wave = square(2 * np.pi * self.wave_hz * t)
        filtered_wave = self.butter_lowpass_filter(square_wave, self.cutoff, self.fs, self.order)
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'odom'
        for i in range(len(filtered_wave)):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = i * self.len / len(filtered_wave)
            pose.pose.position.y = filtered_wave[i]
            path.poses.append(pose)
        self.pub_path.publish(path)


    def __call__(self):
        rospy.Timer(rospy.Duration(1.0/self.hz), self.timer_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        reference_path_creator = ReferencePathCreator()
        reference_path_creator()
    except rospy.ROSInterruptException:
        pass