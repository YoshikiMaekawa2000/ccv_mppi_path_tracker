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

//Kinematic model
#include "ccv_mppi_path_tracker/diff_drive.h"

class CCVMPPIPathTracker
{
public:
    CCVMPPIPathTracker();
    void run();
private:
    ros::NodeHandle nh_;
    // publishers 
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist cmd_vel_;
    // subscribers
    ros::Subscriber path_sub_;
    ros::Subscriber joint_state_sub_;
    nav_msgs::Path path_;
    sensor_msgs::JointState joint_state_;
    // confirmations
    ros::Publisher ref_path_pub_;
    ros::Publisher candidate_path_pub_;
    ros::Publisher optimal_path_pub_;
    ros::Publisher path_pub_;
    nav_msgs::Path ref_path_;
    visualization_msgs::MarkerArray candidate_path_marker_;
    nav_msgs::Path optimal_path_;
    visualization_msgs::Marker path_marker_;
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
    double v_max_;
    double w_max_;
    double v_min_;
    double w_min_;
    double v_ref_;
    double lambda_;

    // MPPI variables
    std::vector<DiffDrive> sample;
    DiffDrive optimal_solution;
    std::vector<double> weights_;


    // Other variables
    double last_time_;
    double current_time_;
    double dt_;
    double resolution_;
    int current_index_;
    std::vector<double> reference_x_;
    std::vector<double> reference_y_;
    std::vector<double> reference_yaw_;

    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;
    std::string KINEMATIC_MODEL;

    bool path_received_;
    bool joint_state_received_;
    bool first_roop_;

    // Other variables and functions for MPPI implementation

    void path_Callback(const nav_msgs::Path::ConstPtr& msg);
    void jointState_Callback(const sensor_msgs::JointState::ConstPtr& msg);
    void get_Transform();
    void sampling();
    void clamp(double &x, double min, double max);
    void predict_States();
    void calculate_Weights();
    void calculate_RefPath();
    int get_CurrentIndex();
    double calculate_Cost(DiffDrive sample);
    void determine_OptimalSolution();
    void publish_CmdVel();

    // confirmations
    void publish_Path();
    void publish_RefPath();
    void publish_OptimalPath();
    void publish_CandidatePath();

};
