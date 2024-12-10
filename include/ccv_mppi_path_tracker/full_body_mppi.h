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
#include <iostream>
#include <fstream>
#include <random>

const double g=9.81;

class DecisionVariables{
    public:
        DecisionVariables();
        void init(int horizon);
        std::vector<double> x_, y_, yaw_, zmp_x, zmp_y;
        std::vector<double> v_, w_, steer_, roll_, pitch_;
};
DecisionVariables::DecisionVariables()
{
};
void DecisionVariables::init(int horizon)
{
    // state
    x_.resize(horizon);
    y_.resize(horizon);
    yaw_.resize(horizon);
    zmp_x.resize(horizon);
    zmp_y.resize(horizon);
    //input
    v_.resize(horizon-1);
    w_.resize(horizon-1);
    steer_.resize(horizon-1);
    roll_.resize(horizon-1);
    pitch_.resize(horizon-1);
    for(int i=0; i<horizon-1; i++)
    {
        x_[i] = 0.0;
        y_[i] = 0.0;
        yaw_[i] = 0.0;
        zmp_x[i] = 0.0;
        zmp_y[i] = 0.0;
        v_[i] = 0.0;
        w_[i] = 0.0;
        steer_[i] = 0.0;
        roll_[i] = 0.0;
        pitch_[i] = 0.0;
    }
    x_[horizon-1] = 0.0;
    y_[horizon-1] = 0.0;
    yaw_[horizon-1] = 0.0;
    zmp_x[horizon-1] = 0.0;
    zmp_y[horizon-1] = 0.0;
}

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
    ros::Publisher pub_roll_pitch_;
    ros::Publisher pub_left_steering_;
    ros::Publisher pub_right_steering_;
    ros::Publisher pub_ref_path_;
    ros::Publisher pub_candidate_path_;
    ros::Publisher pub_optimal_path_;
    geometry_msgs::Twist cmd_vel_;
    // ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    sq2_ccv_roll_pitch_msgs::RollPitch cmd_roll_pitch_;
    std_msgs::Float64 steering_l_, steering_r_;
    nav_msgs::Path ref_path_;
    visualization_msgs::MarkerArray candidate_path_marker_;
    nav_msgs::Path optimal_path_;
    // subscribers
    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_link_states_;
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_;
    sensor_msgs::JointState joint_state_;
    gazebo_msgs::LinkStates link_states_;


    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    geometry_msgs::TransformStamped transform_msg_;

    void get_CurrentState(double current_drive_accel);
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
    void publish_CandidatePath();
    void publish_RefPath();

    void clamp(double &v, double min, double max);
    void predict_NextState(DecisionVariables &sample, int t);
    double calc_ZMP(double lean_angle, double accel);
    void calc_RefPath();
    double calc_Cost(DecisionVariables sample);
    double calc_MinDistance(double x, double y, std::vector<double> x_ref, std::vector<double> y_ref);
    int get_CurrentIndex();

    void save_Data();


    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;

    // MPPI param
    int horizon_;
    double num_samples_;
    double control_noise_;
    double exploration_noise_;
    double lambda_;
    double v_max_, w_max_, steer_max_, roll_max_, pitch_max_;
    double v_min_, w_min_, steer_min_, roll_min_, pitch_min_;
    double v_ref_;
    double path_weight_;
    double v_weight_;
    double zmp_weight_;

    std::vector<DecisionVariables> sample;
    DecisionVariables optimal_solution;
    DecisionVariables current_state_;
    std::vector<double> weights_;

    double current_centripetal_accel_;
    double last_time_;
    double current_time_;
    double last_v_;
    double current_v;
    double last_roll_input_;
    double last_pitch_input_;
    bool path_received_;
    bool odom_received_;
    bool joint_state_received_;
    bool link_states_received_ = false;
    bool first_roop_;

    bool forward_=true;
    bool forward_leaning_=true;

    double dt_;
    double resolution_;
    int current_index_;
    std::vector<double> x_ref_, y_ref_, yaw_ref_;
    double pitch_offset_;
    double tread_;
    double wheel_radius_;
    double weight_;
    double l_center_of_mass_; //回転中心から重心までの距離
    double h_base; //回転中心から地面までの高さ．
};


    
