#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include <random>
#include <iostream>
#include <fstream>
#include "spline.h"


class RobotStates{
    public:
        RobotStates();
        void init(int horizon);
        std::vector<double> x_, y_, yaw_;  // state
        std::vector<double> v_, w_, steer_;  // control variables
};
RobotStates::RobotStates()
{
}

void RobotStates::init(int horizon)
{
    // state
    x_.resize(horizon);
    y_.resize(horizon);
    yaw_.resize(horizon);

    v_.resize(horizon-1);
    w_.resize(horizon-1);
    steer_.resize(horizon-1);
    for(int i=0; i<horizon-1; i++)
    {
        x_[i] = 0.0;
        y_[i] = 0.0;
        yaw_[i] = 0.0;
        v_[i] = 0.0;
        steer_[i] = 0.0;
    }
    x_[horizon-1] = 0.0;
    y_[horizon-1] = 0.0;
    yaw_[horizon-1] = 0.0;
}

class SteeringDiffDriveMPPI
{
public:
    SteeringDiffDriveMPPI();
    void run();
private:
    ros::NodeHandle nh_;
    // publishers
    ros::Publisher pub_cmd_vel_;
    geometry_msgs::Twist cmd_vel_;
    ros::Publisher pub_cmd_pos_;
    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    // subscribers
    ros::Subscriber sub_joint_state_;
    double current_steer_r_, current_steer_l_;
    ros::Subscriber sub_path_;
    nav_msgs::Path path_;
    // for debug
    ros::Publisher pub_ref_path_;
    nav_msgs::Path ref_path_;
    ros::Publisher pub_candidate_path_;
    visualization_msgs::MarkerArray candidate_path_marker_;
    ros::Publisher pub_optimal_path_;
    nav_msgs::Path optimal_path_;
    ros::Publisher pub_fitting_circle_;
    visualization_msgs::Marker fitting_circle_;
    // tf
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    geometry_msgs::TransformStamped transform_msg_;
    // poses
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped previous_pose_;

    // MPPI parameters
    int horizon_;
    double num_samples_;
    double control_noise_;
    double exploration_noise_;
    double v_max_, w_max_, steer_max_;
    double v_min_, w_min_, steer_min_;
    double v_ref_;
    double lambda_;
    double path_weight_;
    double v_weight_;

    // MPPI variables
    std::vector<RobotStates> sample;
    RobotStates optimal_solution;
    std::vector<double> weights_;

    // Other variables and parameters
    double last_time_;
    double current_time_;
    double dt_;
    double resolution_;
    int current_index_;
    std::vector<double> x_ref_, y_ref_, yaw_ref_;
    double pitch_offset_;
    double tread_;
    double wheel_radius_;

    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;

    bool path_received_;
    bool joint_state_received_;
    bool first_roop_;
    bool first_save_;
    std::ofstream ofs;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void get_Transform();
    void sampling();
    void clamp(double &v, double min, double max);
    void predict_States();
    void calc_Weights();
    double calc_Cost(RobotStates sample);
    double calc_TurningRadius(double steer_r, double steer_l);
    double calc_Direction(double R, double steer_r, double steer_l);
    double calc_Omega(double vr, double vl, double steer_r, double steer_l);
    void check_Inside(double &steer_in, double &steer_out, double steer_r, double steer_l);
    void predict_NextState(RobotStates &sample, int t);
    void adjust_Input(RobotStates &sample);
    std::string check_State(double steer_r, double steer_l);
    void check_Samples();
    void calc_RefPath();
    double calc_MinDistance(double x, double y, std::vector<double> x_ref, std::vector<double> y_ref);
    int get_CurrentIndex();
    // double calculate_Cost(DiffDrive sample);
    void determine_OptimalSolution();
    void publish_CmdVel();
    void publish_CmdPos();

    // for debug
    void publish_Path();
    void publish_RefPath();
    void publish_OptimalPath();
    void publish_CandidatePath();
    void save_Data();

    
};

