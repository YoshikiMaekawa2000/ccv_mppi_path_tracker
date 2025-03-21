#!/usr/bin/python3
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from ccv_dynamixel_msgs.msg import CmdPoseByRadian
from sq2_ccv_roll_pitch_msgs.msg import RollPitch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import TransformException
import datetime
import subprocess
import csv
import numpy as np



class RecordState:
    def __init__(self):
        rospy.init_node('record_state')
        self.sub_odom = rospy.Subscriber('/sq2_ccv/diff_drive_steering_controller/odom', Odometry, self.odom_callback)
        self.sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelstate_callback)
        self.sub_link_state = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkstate_callback)
        self.sub_dynamixel_state = rospy.Subscriber('/local/cmd_pos', CmdPoseByRadian, self.dynamixelstate_callback)
        self.sub_cmd_vel = rospy.Subscriber('/sq2_ccv/diff_drive_steering_controller/cmd_vel', Twist, self.cmd_vel_callback)
        self.sub_joint_state = rospy.Subscriber('/sq2_ccv/joint_states', JointState, self.joint_state_callback)
        self.sub_roll_pitch = rospy.Subscriber('/sq2_ccv/roll_pitch_controller/roll_pitch', RollPitch, self.roll_pitch_callback)
        
        #記録するデータ：x, y(真値とtf値)， v,δ_r, δ_l(真値と指令値)，目標軌道
        self.path = Path()
        self.state = ModelStates()
        self.link_state = LinkStates()
        self.dynamixel_state = CmdPoseByRadian()
        self.cmd_vel = Twist()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.state_file_name = ""
        self.csv_file = None
        self.path_flag = False
        self.csv_writer = None

        rospy.on_shutdown(self.record_path)
        
##################callback関数##################
    def odom_callback(self, msg):
        self.odom = msg
        self.odom_flag = True
    def modelstate_callback(self, msg):
        self.state = msg
        self.state_flag = True
    def linkstate_callback(self, msg):
        self.link_state = msg
        self.link_state_flag = True
    def dynamixelstate_callback(self, msg):
        self.dynamixel_state = msg
        self.dynamixel_state_flag = True
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_flag = True
    def joint_state_callback(self, msg):
        self.joint_state = msg
        self.joint_state_flag = True
    def roll_pitch_callback(self, msg):
        self.roll_pitch = msg
        self.roll_pitch_flag = True
##############################################
    def get_true_pose(self):
        for i in range(len(self.state.name)):
            if(self.state.name[i] == "sq2_ccv"):
                return self.state.pose[i].position.x, self.state.pose[i].position.y
    def get_tf_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            return transform.transform.translation.x, transform.transform.translation.y
        except TransformException as e:
            rospy.logerr(e)
            return None



    def record_path(self):
        for i, pose in enumerate(self.path.poses):
            self.csv_writer.writerow(["", "", "", "", "", "", "", pose.pose.position.x, pose.pose.position.y])
        self.csv_file.close()
            
        
    def __call__(self):
        rate = rospy.Rate(10)
        date = datetime.datetime.now()
        date = str(date.strftime("%Y%m%d_%H%M%S"))
        self.file_name = "../log/full_body/" + date + ".csv"
        self.csv_file = open(self.file_name, "w", newline="")

        self.csv_writer = csv.writer(self.csv_file)
        # self.csv_writer.writerow(["time", "x", "y", "x_tf", "y_tf", "v", "cmd_v", "path_x", "path_y"])
        self.csv_writer.writerow(["time", "front_pitch", "pitch_cmd", "odom_v", "cmd_v", "estimated_a", "real_a"])
        start = rospy.get_time()
        last_v = 0
        last_time = rospy.get_time()
        while not rospy.is_shutdown():
            # if(tf_x < 15):
                front_pitch = self.joint_state.position[2]
                pitch_cmd = self.roll_pitch.pitch
                odom_v = self.odom.twist.twist.linear.x
                cmd_v = self.cmd_vel.linear.x
                estimated_a = odom_v - last_v
                last_v = odom_v

                true_v = np.sqrt(self.state.twist[1].linear.x**2 + self.state.twist[1].linear.y**2)
                if(self.state.twist[1].linear.x < 0):
                    true_v = -true_v
                
                self.csv_writer.writerow([rospy.get_time() - start, front_pitch, pitch_cmd, odom_v, cmd_v, "", ""])
                # self.csv_writer.writerow([true_x, true_y, tf_x, tf_y, true_v, cmd_v, "", ""])
                rate.sleep()
            # else:
                # pass
            

if __name__ == '__main__':
    try:
        record_state = RecordState()
        record_state()
    except rospy.ROSInterruptException:
        pass
        