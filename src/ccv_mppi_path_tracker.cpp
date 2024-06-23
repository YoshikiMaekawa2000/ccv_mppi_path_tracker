
#include "ccv_mppi_path_tracker/ccv_mppi_path_tracker.h"
// #include "ccv_mppi_path_tracker/diff_drive.h"

CCVMPPIPathTracker::CCVMPPIPathTracker()
    : nh_("~"), path_received_(false), joint_state_received_(false), first_roop_(true)
{
    // publishers 
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/sq2_ccv/diff_drive_steering_controller/cmd_vel", 1);
    // confirmations
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/ref_path", 1);
    candidate_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ccv_mppi_path_tracker/candidate_path", 1);
    optimal_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/ccv_mppi_path_tracker/optimal_path", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/path", 1);

    // subscribers
    path_sub_ = nh_.subscribe("/reference_path_creater/path", 1, &CCVMPPIPathTracker::path_Callback, this);
    joint_state_sub_ = nh_.subscribe("/sq2_ccv/joint_states", 1, &CCVMPPIPathTracker::jointState_Callback, this);

    nh_.param("dt", dt_, 0.1);
    nh_.param("horizon", horizon_, 10);
    nh_.param("num_samples", num_samples_, 100.0);
    nh_.param("control_noise", control_noise_, 0.1);
    nh_.param("lambda", lambda_, 1.0);
    nh_.param("v_max", v_max_, 1.0);
    nh_.param("w_max", w_max_, 1.0);
    nh_.param("v_min", v_min_, -1.0);
    nh_.param("w_min", w_min_, -1.0);
    nh_.param("v_ref", v_ref_, 1.2);
    nh_.param("resolution", resolution_, 0.1);
    nh_.param("exploration_noise", exploration_noise_, 0.1);
    nh_.param("world_frame", WORLD_FRAME, std::string("odom"));
    nh_.param("robot_frame", ROBOT_FRAME, std::string("base_link"));
    nh_.param("kinematic_model", KINEMATIC_MODEL, std::string("diff_drive"));

    // Initialize sample
    sample.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++)
    {
        sample[i].init(horizon_);
    }
    // Initialize optimal_solution
    optimal_solution.init(horizon_);
    // Initialize reference path
    reference_x_.resize(horizon_);
    reference_y_.resize(horizon_);
    reference_yaw_.resize(horizon_);
    // Initialize weights
    weights_.resize(num_samples_);
}

void CCVMPPIPathTracker::path_Callback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
    
    if (!path_received_)
    {
        path_received_ = true;
    }
}
void CCVMPPIPathTracker::jointState_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state_ = *msg;
    if (!joint_state_received_)
    {
        joint_state_received_ = true;
    }
}

void CCVMPPIPathTracker::get_Transform()
{
    try
    {
        listener_.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), transform_);
        tf::transformStampedTFToMsg(transform_, transform_msg_);
        current_pose_.header = transform_msg_.header;
        current_pose_.pose.position.x = transform_msg_.transform.translation.x;
        current_pose_.pose.position.y = transform_msg_.transform.translation.y;
        current_pose_.pose.orientation = transform_msg_.transform.rotation;
        // std::cout << "===Transformed===" << std::endl;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // test
    //  std::cout << "===Current pose===" << std::endl;
    //  std::cout << "x: " << current_pose_.pose.position.x << std::endl;
    //  std::cout << "y: " << current_pose_.pose.position.y << std::endl;
    //  std::cout << "yaw: " << tf::getYaw(current_pose_.pose.orientation) << std::endl;
}

void CCVMPPIPathTracker::clamp(double &x, double min, double max)
{
    if (x < min)
    {
        x = min;
    }
    else if (x > max)
    {
        x = max;
    }
}

void CCVMPPIPathTracker::sampling()
{
    // Add Gaussian noise to control inputs
    std::random_device rnd;
    std::mt19937 mt(rnd());
    for (int t = 0; t < horizon_; t++)
    {
        std::normal_distribution<> norm_v(optimal_solution.v_[t], control_noise_);
        std::normal_distribution<> norm_w(optimal_solution.w_[t], control_noise_);
        for (int i = 0; i < num_samples_; i++)
        {
            sample[i].v_[t] = norm_v(mt);
            sample[i].w_[t] = norm_w(mt);
            // Clamp control inputs
            clamp(sample[i].v_[t], v_min_, v_max_);
            clamp(sample[i].w_[t], w_min_, w_max_);
        }
    }
}
void CCVMPPIPathTracker::publish_Path(){
    path_pub_.publish(path_marker_);
}


void CCVMPPIPathTracker::publish_CmdVel()
{
    // cmd_vel_.linear.x = 1.0;
    // cmd_vel_.angular.z = 0.2;
    // cmd_vel_.angular.z = 0;

    cmd_vel_.linear.x = optimal_solution.v_[0];
    cmd_vel_.angular.z = optimal_solution.w_[0];
    cmd_vel_pub_.publish(cmd_vel_);
}

void CCVMPPIPathTracker::predict_States()
{
    for (int i = 0; i < num_samples_; i++)
    {
        sample[i].x_[0] = current_pose_.pose.position.x;
        sample[i].y_[0] = current_pose_.pose.position.y;
        sample[i].yaw_[0] = tf::getYaw(current_pose_.pose.orientation);
    }
    // Predict states
    for (int i = 0; i < num_samples_; i++)
    {
        for (int t = 0; t < horizon_ - 1; t++)
        {
            sample[i].x_[t + 1] = sample[i].x_[t] + sample[i].v_[t] * cos(sample[i].yaw_[t]) * dt_;
            sample[i].y_[t + 1] = sample[i].y_[t] + sample[i].v_[t] * sin(sample[i].yaw_[t]) * dt_;
            sample[i].yaw_[t + 1] = sample[i].yaw_[t] + sample[i].w_[t] * dt_;
        }
    }
    // test
    // std::cout << "===Predicted states===" << std::endl;

}

int CCVMPPIPathTracker::get_CurrentIndex()
{
    int index = 0;
    double min_distance = 100.0;
    for (int i = 0; i < path_.poses.size(); i++)
    {
        double distance = sqrt(pow(current_pose_.pose.position.x - path_.poses[i].pose.position.x, 2) + pow(current_pose_.pose.position.y - path_.poses[i].pose.position.y, 2));
        if (distance < min_distance)
        {
            min_distance = distance;
            index = i;
        }
    }
    return index;
}

void CCVMPPIPathTracker::publish_RefPath()
{
    ref_path_.header.frame_id = WORLD_FRAME;
    ref_path_.header.stamp = ros::Time::now();
    ref_path_.poses.resize(horizon_);
    for(int i=0; i<horizon_; i++)
    {
        ref_path_.poses[i].pose.position.x = reference_x_[i];
        ref_path_.poses[i].pose.position.y = reference_y_[i];
        ref_path_.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(reference_yaw_[i]);
    }
    ref_path_pub_.publish(ref_path_);
}
void CCVMPPIPathTracker::calculate_RefPath()
{
    // Calculate reference path
    current_index_ = get_CurrentIndex();
  
    //あとで直す
    // double step = v_ref_ * dt_ / resolution_;
    int step = 1;
    for(int i=0; i<horizon_; i++)
    {
        int index = current_index_ + i * step;
        std::cout << "index: " << index << std::endl;
        if(index < path_.poses.size())
        {
            reference_x_[i] = path_.poses[index].pose.position.x;
            reference_y_[i] = path_.poses[index].pose.position.y;
        }
        else
        {
            reference_x_[i] = path_.poses[path_.poses.size()-1].pose.position.x;
            reference_y_[i] = path_.poses[path_.poses.size()-1].pose.position.y;
        }
    }
    for(int i=0; i<horizon_-1; i++)
    {
        reference_yaw_[i] = atan2(reference_y_[i+1]-reference_y_[i], reference_x_[i+1]-reference_x_[i]);
    }

    // Publish reference path
    publish_RefPath();
}

double CCVMPPIPathTracker::calculate_Cost(DiffDrive sample)
{
    // Calculate cost
    double cost = 0.0;
    for(int t=0; t<horizon_; t++)
    {
        double dx = sample.x_[t] - reference_x_[t];
        double dy = sample.y_[t] - reference_y_[t];
        double dyaw = sample.yaw_[t] - reference_yaw_[t];
        cost += dx*dx + dy*dy + dyaw*dyaw;
    }
    return cost;
}
void CCVMPPIPathTracker::calculate_Weights()
{
    //calcurate reference path
    calculate_RefPath();

    // Calculate weights
    double sum = 0.0;
    for(int i=0; i<num_samples_; i++)
    {
        // Calculate cost
        double cost = calculate_Cost(sample[i]);

        weights_[i] = exp(-cost / lambda_);
        sum += weights_[i];
    }
    // Normalize weights
    for(int i=0; i<num_samples_; i++)
    {
        weights_[i] /= sum;
    }
}

void CCVMPPIPathTracker::determine_OptimalSolution()
{
    // Determine optimal solution
    for(int t=0; t<horizon_; t++)
    {
        optimal_solution.v_[t] = 0.0;
        optimal_solution.w_[t] = 0.0;
        for(int i=0; i<num_samples_; i++)
        {
            optimal_solution.v_[t] += weights_[i] * sample[i].v_[t];
            optimal_solution.w_[t] += weights_[i] * sample[i].w_[t];
        }
    }
    // test
    // std::cout << "===Optimal solution===" << std::endl;
    // for(int t=0; t<horizon_; t++)
    // {
    //     std::cout << "v: " << optimal_solution.v_[t] << std::endl;
    //     std::cout << "w: " << optimal_solution.w_[t] << std::endl;
    // }
}

void CCVMPPIPathTracker::run()
{
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (path_received_ && joint_state_received_)
        {
            if (first_roop_)
            {
                last_time_ = ros::Time::now().toSec();
                first_roop_ = false;
            }
            else
            {
                current_time_ = ros::Time::now().toSec();
                dt_ = current_time_ - last_time_;
                last_time_ = current_time_;

                // std::cout << "===Path and joint state received===" << std::endl;
                // Implement MPPI here
                // 0. Get transform
                get_Transform();
                // 1. Sample control inputs
                sampling();
                // 2. Predict states
                predict_States();
                // 3. Calculate weights
                calculate_Weights();
                // 4. determine optimal solution
                determine_OptimalSolution();
                // 5. Publish path and cmd_vel
                publish_Path();
                publish_CmdVel();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ccv_mppi_path_tracker");
    CCVMPPIPathTracker ccv_mppi_path_tracker;
    ccv_mppi_path_tracker.run();
    return 0;
}