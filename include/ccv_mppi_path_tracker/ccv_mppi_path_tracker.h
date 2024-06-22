#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>

class CCVMPPIPathTracker
{
public:
    CCVMPPIPathTracker();
    void run();
private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber joint_state_sub_;

    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    geometry_msgs::TransformStamped transform_msg_;

    nav_msgs::Path path_;
    sensor_msgs::JointState joint_state_;
    geometry_msgs::PoseStamped current_pose_;

    // MPPI parameters
    double dt_;
    double horizon_;
    double num_samples_;
    double control_noise_;
    double exploration_noise_;

    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;

    bool path_received_;
    bool joint_state_received_;
    bool transformed_;

    // Other variables and functions for MPPI implementation

    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void getTransform();
    void sampling();
    void updatePose();
    void calc_weights();
    void updatePath();
};