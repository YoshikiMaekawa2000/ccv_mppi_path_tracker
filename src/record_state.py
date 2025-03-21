#!/usr/bin/python3
import rospy
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from ccv_dynamixel_msgs.msg import CmdPoseByRadian
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException
import datetime
import subprocess
import csv
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RecordState:
    def __init__(self):
        rospy.init_node('record_state')
        self.sub_path = rospy.Subscriber('/reference_path', Path, self.path_callback)
        self.sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelstate_callback)
        self.sub_link_state = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkstate_callback)
        self.sub_dynamixel_state = rospy.Subscriber('/local/cmd_pos', CmdPoseByRadian, self.dynamixelstate_callback)
        self.sub_cmd_vel = rospy.Subscriber('/sq2_ccv/diff_drive_steering_controller/cmd_vel', Twist, self.cmd_vel_callback)
        self.sub_joint_state = rospy.Subscriber('/sq2_ccv/joint_states', JointState, self.joint_state_callback)
        self.sub_true_zmp = rospy.Subscriber("/ccv_mppi_path_tracker/true_zmp",  Float64, self.true_zmp_callback)
        self.sub_zmp_y = rospy.Subscriber("/ccv_mppi_path_tracker/zmp_y", Float64, self.zmp_y_callback)

        #記録するデータ：x, y(真値とtf値)， v,δ_r, δ_l(真値と指令値)，目標軌道
        self.path = Path()
        self.state = ModelStates()
        self.link_state = LinkStates()
        self.dynamixel_state = CmdPoseByRadian()
        self.cmd_vel = Twist()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_state = JointState()
        self.true_zmp = Float64()
        self.zmp_y = Float64()

        self.state_file_name = ""
        self.csv_file = None
        self.path_flag = False
        self.csv_writer = None

        rospy.on_shutdown(self.record_path)

##################callback関数##################
    def path_callback(self, msg):
        if(not self.path_flag):
            self.path = msg
            self.path_flag = True
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
    def true_zmp_callback(self, msg):
        self.true_zmp = msg
        self.true_zmp_flag = True
    def zmp_y_callback(self, msg):
        self.zmp_y = msg
        self.zmp_y_flag = True
##############################################
    def get_node(self):
        nodes = subprocess.check_output(["rosnode", "list"]).splitlines()
        pure_pursuit = "/pure_pursuit_steering"
        mppi = "/steering_diff_drive_mppi"
        d_mppi = "/diff_drive_mppi"
        full = "/full_body_mppi"
        for node in nodes:
            node = node.decode("utf-8")
            if(node == pure_pursuit):
                return "../log/pure_pursuit/robo_sym/"
            elif(node == mppi):
                return "../log/mppi/master_thesis/"
            elif(node == d_mppi):
                return "../log/mppi/master_thesis/"
            elif(node == full):
                return "../log/full_body/robo_sym/"

    def get_true_pose_and_omega(self):
        for i in range(len(self.state.name)):
            if(self.state.name[i] == "sq2_ccv"):
                return self.state.pose[i].position.x, self.state.pose[i].position.y, self.state.twist[i].angular.z, self.state.pose[i].orientation.z
    def get_tf_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            return transform.transform.translation.x, transform.transform.translation.y
        except TransformException as e:
            rospy.logerr(e)
            return None
    def get_steering_angle(self):
        for i in range(len(self.joint_state.name)):
            if(self.joint_state.name[i] == "right_steering_joint"):
                steer_r = self.joint_state.position[i]
            elif(self.joint_state.name[i] == "left_steering_joint"):
                steer_l = self.joint_state.position[i]
        return steer_r, steer_l

    def record_path(self):
        for i, pose in enumerate(self.path.poses):
            self.csv_writer.writerow(["", "", "", "", "", "", "", "", "", "", "", "", "", "", pose.pose.position.x, pose.pose.position.y])
        self.csv_file.close()


    def __call__(self):
        rate = rospy.Rate(10)
        date = datetime.datetime.now()
        date = str(date.strftime("%Y%m%d_%H%M%S"))
        self.file_name = self.get_node() + date + ".csv"
        self.csv_file = open(self.file_name, "w", newline="")

        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "x", "y", "omega", "yaw", "x_tf", "y_tf", "v", "cmd_v", "steer_r", "steer_l","roll", "true_zmp", "zmp_y", "path_x", "path_y"])
        start = rospy.get_time()
        while not rospy.is_shutdown():
            # if(tf_x < 15):
                true_x, true_y, true_omega, true_yaw= self.get_true_pose_and_omega()
                tf_x, tf_y = self.get_tf_pose()
                true_v = np.sqrt(self.state.twist[1].linear.x**2 + self.state.twist[1].linear.y**2)
                steer_r, steer_l = self.get_steering_angle()
                cmd_v = self.cmd_vel.linear.x
                if(self.state.twist[1].linear.x < 0):
                    true_v = -true_v
                self.csv_writer.writerow([rospy.get_time() - start, true_x, true_y, true_omega, true_yaw, tf_x, tf_y, true_v, cmd_v, steer_r, steer_l, self.dynamixel_state.roll, self.true_zmp.data, self.zmp_y.data, "", ""])
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

