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
#include <queue>

const double g=9.81;

class RobotStates{
    public:
        RobotStates(size_t horizon){
            size_t i_horizon;
            if(horizon >= 2) i_horizon = horizon - 1;
            else i_horizon = horizon;

            x_.resize(horizon);
            y_.resize(horizon);
            yaw_.resize(horizon);
            zmp_x_.resize(horizon);
            zmp_y_.resize(horizon);
            roll_.resize(horizon);
            pitch_.resize(horizon);
            v_.resize(i_horizon);
            w_.resize(i_horizon);
            direction_.resize(i_horizon);
            roll_v_.resize(i_horizon);
            pitch_v_.resize(i_horizon);
            
        }
        RobotStates(){
        }
        std::vector<double> x_, y_, yaw_, roll_, pitch_, zmp_x_, zmp_y_; //state
        std::vector<double> v_, w_, direction_, roll_v_, pitch_v_;  //input
};

// class AccelerationBuffer {
//     private:
//         struct DataPoint {
//             double velocity; 
//             double time;
//         };
//         std::vector<DataPoint> buffer; 
//         size_t index;                       
//         size_t max_size;                      
//     public:
//         AccelerationBuffer(size_t size) : index(0), max_size(size) {
//             buffer.resize(size);
//         }
//         void addVelocity(double velocity, double timestamp) {
//             buffer[index] = {velocity, timestamp};
//             index = (index + 1) % max_size;
//         }
//         double calculateAverageAcceleration(){
//             double total_acceleration = 0.0;

//             size_t min_time_index = 0;
//             for (size_t i = 1; i < max_size; ++i) {
//                 if (buffer[i].time < buffer[min_time_index].time) {
//                     min_time_index = i;
//                 }
//             }
//             for(size_t i = 0; i < max_size; ++i){
//                 if(i != min_time_index){
//                     total_acceleration += (buffer[i].velocity - buffer[min_time_index].velocity) / (buffer[i].time - buffer[min_time_index].time);
//                 }
//             }
//             return total_acceleration / (max_size - 1);
//         }
// };

class AverageAccelerationCalculator {
private:
    std::queue<std::pair<double, double>> buffer; // Buffer to store {time, velocity} pairs
    size_t bufferSize; // Maximum size of the buffer

public:
    // Constructor to initialize the buffer size
    AverageAccelerationCalculator(size_t size) : bufferSize(size) {}

    // Function to add velocity and time to the buffer
    void addVelocity(double time, double velocity) {
        if (buffer.size() == bufferSize) {
            buffer.pop(); // Remove the oldest entry if the buffer is full
        }
        buffer.emplace(time, velocity);
    }

    // Function to calculate average acceleration
    double calculateAverageAcceleration() {
        if (buffer.size() < 2) {
            throw std::runtime_error("Not enough data to calculate average acceleration.");
        }

        auto first = buffer.front();
        auto last = buffer.back();

        double initialTime = first.first;
        double initialVelocity = first.second;
        double finalTime = last.first;
        double finalVelocity = last.second;

        if (finalTime == initialTime) {
            throw std::runtime_error("Invalid data: Time difference is zero.");
        }

        return (finalVelocity - initialVelocity) / (finalTime - initialTime);
    }
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
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_;
    sensor_msgs::JointState joint_state_;
    gazebo_msgs::LinkStates link_states_;


    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    // tf::StampedTransform transform_com;
    geometry_msgs::TransformStamped transform_msg_;

    void get_CurrentState(double drive_accel, double roll_accel, double pitch_accel);
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
    void predict_NextState(RobotStates &sample, int t);
    double calc_ZMP(double accel, double CoM_z, double CoM_ground);
    void calc_RefPath();
    double calc_Cost(RobotStates sample);
    double calc_MinDistance(double x, double y, std::vector<double> x_ref, std::vector<double> y_ref);
    int get_CurrentIndex();

    void save_Data();


    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;
    std::string CoM_FRAME;

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

    std::vector<RobotStates> sample;
    RobotStates optimal_solution_;
    RobotStates current_state_;
    std::vector<double> weights_;

    double current_centripetal_accel_;
    bool path_received_;
    bool odom_received_;
    bool joint_state_received_;
    bool link_states_received_ = false;
    bool first_roop_;

    bool forward_leaning_=true;
    bool forward_ = true;

    double dt_;
    size_t buffer_size_;
    double resolution_;
    int current_index_;
    std::vector<double> x_ref_, y_ref_, yaw_ref_;
    double pitch_offset_;
    double tread_;
    double wheel_radius_;
    double weight_;
    double base2CoM; //回転中心から重心までの距離
    double ground2base; //回転中心から地面までの高さ．

    double ave_com_x = 0.0;
};


    
