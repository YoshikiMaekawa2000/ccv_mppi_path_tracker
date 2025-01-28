#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include <sq2_ccv_roll_pitch_msgs/RollPitch.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <fstream>
#include <random>
#include <queue>
#include <gazebo_msgs/GetLinkState.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>

Eigen::Vector3d gravity_(0.0, 0.0, -9.8);
const tf::Vector3 gravity(0.0, 0.0, -9.8);
const double g=-9.81;

class RobotStates{
    public:
        RobotStates(size_t horizon){
            size_t input_h = horizon;
            size_t variable_h = horizon;
            if(horizon >= 3){
                input_h = horizon-1;
                variable_h = horizon-2;
            }
            //states
            x_.resize(horizon);
            y_.resize(horizon);
            yaw_.resize(horizon);
            roll_.resize(horizon);
            pitch_.resize(horizon);
            //input
            v_.resize(input_h);
            w_.resize(input_h);
            direction_.resize(input_h);
            roll_v_.resize(input_h);
            pitch_v_.resize(input_h);
            //variables for calc ZMP
            zmp_x_.resize(variable_h);
            zmp_y_.resize(variable_h);  
            
        }
        RobotStates(){
        }
        std::vector<double> x_, y_, yaw_, roll_, pitch_; //states
        std::vector<double> v_, w_, direction_, roll_v_, pitch_v_;  //input
        std::vector<double> zmp_x_, zmp_y_; //zmp
};


class FullBodyMPPI
{
public:
    FullBodyMPPI();
    void run();
private:
    ros::NodeHandle nh_;
    // publishers
    ros::Publisher pub_cmd_vel_;
    // ros::Publisher pub_cmd_pos_;
    ros::Publisher pub_cmd_pos_;
    ros::Publisher pub_ref_path_;
    ros::Publisher pub_candidate_path_;
    ros::Publisher pub_optimal_path_;
    geometry_msgs::Twist cmd_vel_;
    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    nav_msgs::Path ref_path_;
    visualization_msgs::MarkerArray candidate_path_marker_;
    nav_msgs::Path optimal_path_;
    // subscribers
    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_link_states_;
    ros::Subscriber sub_imu_;
    // ros::Subscriber sub_wrench_;
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_;
    sensor_msgs::JointState joint_state_;
    gazebo_msgs::LinkStates link_states_;
    tf::Quaternion imu_orientation_;
    tf::Vector3 accel_base;
    double imu_roll_, imu_pitch_, imu_yaw_;
    double accel_x, accel_y, accel_z;
    sensor_msgs::Imu filterd_imu_;
    sensor_msgs::Imu last_imu_;

    std::vector<std::string> force_sensor_topic_;
    std::vector<ros::Subscriber> sub_force_sensor_;
    std::map<std::string, geometry_msgs::WrenchStamped> force_sensor_data_;

    ros::Publisher pub_zmp_y_;
    std_msgs::Float64 zmp_y_;
    ros::Publisher pub_true_zmp_;
    std_msgs::Float64 true_zmp_;

    ros::Publisher pub_drive_accel_;
    std_msgs::Float64 drive_accel_;


    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    // tf::StampedTransform transform_com;
    geometry_msgs::TransformStamped transform_msg_;

    void calc_true_ZMP();
    Eigen::Vector3d computeZMPfromModel(Eigen::Vector3d r, Eigen::Vector3d aG, Eigen::Vector3d alpha);

    // void get_CurrentState(double drive_accel, double roll_accel, double pitch_accel);
    void get_CurrentState();
    void sampling();
    void predict_States();
    void calc_Weights();
    void determine_OptimalSolution();
    void publish_CmdVel();
    void publish_CmdPos();
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void publish_CandidatePath();
    void publish_RefPath();
    Eigen::Vector3d calc_HO(Eigen::Vector3d V, Eigen::Vector3d r, Eigen::Vector3d omega);

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, const std::string& topic);

    void clamp(double &v, double min, double max);
    void predict_NextState(RobotStates &sample, int t);
    void calc_RefPath();
    double calc_Cost(RobotStates sample);
    double calc_MinDistance(double x, double y, std::vector<double> x_ref, std::vector<double> y_ref);
    int get_CurrentIndex();

    void save_Data();


    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;
    std::string IMU_FRAME;
    std::string RIGHT_WHEEL = "right_wheel_link";
    std::string LEFT_WHEEL = "left_wheel_link";

    // MPPI param
    int horizon_;
    int num_samples_;
    double control_noise_;
    double exploration_noise_;
    double lambda_;
    double v_max_, w_max_, steer_max_, roll_max_, pitch_max_, roll_v_max_, pitch_v_max_;
    double v_min_, w_min_, steer_min_, roll_min_, pitch_min_, roll_v_min_, pitch_v_min_;
    double v_ref_;
    double path_weight_;
    double v_weight_;
    double zmp_weight_;
    double roll_v_weight_;
    bool off_;

    std::vector<RobotStates> sample;
    RobotStates optimal_solution_;
    RobotStates current_state_;
    std::vector<double> weights_;

    double current_centripetal_accel_;
    bool path_received_ = false;
    bool odom_received_ = false;
    bool joint_state_received_ = false;
    bool link_states_received_ = false;
    bool first_roop_ = true;
    bool imu_received_ = false;

    bool forward_leaning_=true;
    bool forward_ = true;

    double dt_;
    double last_time_;
    size_t buffer_size_;
    double resolution_;
    int current_index_;
    std::vector<double> x_ref_, y_ref_, yaw_ref_;
    double pitch_offset_;
    double tread_;
    double wheel_radius_;
    double weight_;
    double base2CoM;
    double ground2base; //回転中心から地面までの高さ．

    double upper_body_radius = 0.11;
    double upper_body_height = 0.8075;
    double upper_body_depth = 0.208;
    double upper_body_width = 0.208;
    double mass = 60.0;
    // double alpha = 0.5; //low pass filter
    double alpha = 0.3; //low pass filter
    

    std::vector<Eigen::Vector3d> contactPositions;
    Eigen::Vector3d base2front_r_caster, base2front_l_caster, base2back_r_caster, base2back_l_caster;
    Eigen::Vector3d base2wheel_r, base2wheel_l;
    Eigen::Matrix3d I_O;
    Eigen::Vector3d z=Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d last_HG=Eigen::Vector3d::Zero();
    // Eigen::Vector3d H_G;

    Eigen::Vector3d true_ZMP;
    double direction_input;
};

