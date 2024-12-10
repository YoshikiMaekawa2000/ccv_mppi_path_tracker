#include "ccv_mppi_path_tracker/full_body_mppi.h"

FullBodyMPPI::FullBodyMPPI()
    : nh_("~"), path_received_(false), odom_received_(false), joint_state_received_(false), first_roop_(true), 
    last_pitch_input_(0.0), last_roll_input_(0.0), tread_(0.501), wheel_radius_(0.1435),
    weight_(60), l_center_of_mass_(0.5735), h_base(0.10)
{
    // publishers
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    // pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);
    pub_roll_pitch_ = nh_.advertise<sq2_ccv_roll_pitch_msgs::RollPitch>("/sq2_ccv/roll_pitch_controller/roll_pitch", 1);
    pub_left_steering_ = nh_.advertise<std_msgs::Float64>("/sq2_ccv/diff_drive_steering_controller/steering_cmd/left", 1);
    pub_right_steering_ = nh_.advertise<std_msgs::Float64>("/sq2_ccv/diff_drive_steering_controller/steering_cmd/right", 1);
    // subscribers
    sub_path_ = nh_.subscribe("/reference_path", 1, &FullBodyMPPI::pathCallback, this);
    sub_odom_ = nh_.subscribe("/odom", 1, &FullBodyMPPI::odomCallback, this);
    sub_joint_state_ = nh_.subscribe("/sq2_ccv/joint_states", 1, &FullBodyMPPI::jointStateCallback, this);
    // sub_link_states_ = nh_.subscribe("/gazebo/link_states", 1, &FullBodyMPPI::linkStatesCallback, this);
    // for debug
    pub_ref_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/ref_path", 1);
    pub_candidate_path_ = nh_.advertise<visualization_msgs::MarkerArray>("/ccv_mppi_path_tracker/candidate_path", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/optimal_path", 1);

    nh_.param("dt", dt_, 0.1);
    nh_.param("horizon", horizon_, 15);
    nh_.param("num_samples", num_samples_, 1000.0);
    nh_.param("control_noise", control_noise_, 0.5);
    nh_.param("lambda", lambda_, 1.0);
    nh_.param("v_max", v_max_, 1.6);
    nh_.param("w_max", w_max_, 1.0);
    nh_.param("steer_max", steer_max_, {30.0*M_PI/180.0});
    nh_.param("roll_max", roll_max_, {30.0*M_PI/180.0});
    nh_.param("pitch_max", pitch_max_, {15.0*M_PI/180.0});
    nh_.param("v_min", v_min_, -1.6);
    nh_.param("w_min", w_min_, -1.0);
    nh_.param("steer_min", steer_min_, {-30.0*M_PI/180.0});
    nh_.param("roll_min", roll_min_, {-30.0*M_PI/180.0});
    nh_.param("pitch_min", pitch_min_, {-15.0*M_PI/180.0});
    nh_.param("pitch_offset", pitch_offset_, {0.0*M_PI/180.0});
    nh_.param("v_ref", v_ref_, 0.8);
    nh_.param("resolution", resolution_, 0.1);
    nh_.param("exploration_noise", exploration_noise_, 0.1);
    nh_.param("world_frame", WORLD_FRAME, std::string("odom"));
    nh_.param("robot_frame", ROBOT_FRAME, std::string("base_link"));
    nh_.param("path_weight", path_weight_, 1.0);
    nh_.param("control_weight", v_weight_, 1.0);
    nh_.param("zmp_weight", zmp_weight_, 100.0);
    
    sample.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++) sample[i].init(horizon_);

    optimal_solution.init(horizon_);
    current_state_.init(1);

    x_ref_.resize(horizon_);
    y_ref_.resize(horizon_);
    yaw_ref_.resize(horizon_);
    weights_.resize(num_samples_);
}
void FullBodyMPPI::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
    if(!path_received_) path_received_ = true;
}

void FullBodyMPPI::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ = *msg;
    if(!odom_received_) odom_received_ = true;
}

void FullBodyMPPI::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state_ = *msg;
    if(!joint_state_received_) joint_state_received_ = true;
    std::cout << "front   pitch: " << joint_state_.position[2]*180/M_PI << std::endl;
}

// void FullBodyMPPI::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg)
// {
//     link_states_ = *msg;
//     if(!link_states_received_) link_states_received_ = true;
//     // std::cout << link_states_.name[4] << std::endl;
//     tf::Matrix3x3 m(tf::Quaternion(link_states_.pose[4].orientation.x, link_states_.pose[4].orientation.y, link_states_.pose[4].orientation.z, link_states_.pose[4].orientation.w));
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     // std::cout << "roll: " << roll*180/M_PI << " pitch: " << pitch*180/M_PI << " yaw: " << yaw*180/M_PI << std::endl;
//     std::cout << "gazebo  pitch: " << pitch*180/M_PI << std::endl;
// }
void FullBodyMPPI::publish_CmdVel()
{
    // std::cout << "v: " << optimal_solution.v_[0] << " w: " << optimal_solution.w_[0] << std::endl;

    cmd_vel_.linear.x = optimal_solution.v_[0];
    cmd_vel_.angular.z = optimal_solution.w_[0];

    // if(forward_){
    //     cmd_vel_.linear.x = odom_.twist.twist.linear.x+0.1;
    //     cmd_vel_.angular.z = 0.0;
    //     if(cmd_vel_.linear.x > v_max_) forward_ = false;
    // }
    // else{
    //     cmd_vel_.linear.x = odom_.twist.twist.linear.x-0.1;
    //     cmd_vel_.angular.z = 0.0;
    //     if(cmd_vel_.linear.x < v_min_) forward_ = true;
    // }
    // optimal_solution.v_[0] = cmd_vel_.linear.x;
    // optimal_solution.w_[0] = cmd_vel_.angular.z;

    // cmd_vel_.linear.x = 0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FullBodyMPPI::publish_CmdPos()
{
    double R = fabs(optimal_solution.v_[0] / optimal_solution.w_[0]);
    double steer_in = std::atan2(R*sin(optimal_solution.steer_[0]), R*cos(optimal_solution.steer_[0]) - tread_/2.0);
    double steer_out = std::atan2(R*sin(optimal_solution.steer_[0]), R*cos(optimal_solution.steer_[0]) + tread_/2.0);

    if(optimal_solution.w_[0] > 0.0){
        steering_l_.data = steer_in;
        steering_r_.data = steer_out;
    }
    else{
        steering_l_.data = steer_out;
        steering_r_.data = steer_in;
    }
    // std::cout << "steer_l: " << steering_l_.data*180/M_PI << " steer_r: " << steering_r_.data*180/M_PI << std::endl;

    cmd_roll_pitch_.roll = 0.0;
    cmd_roll_pitch_.pitch = 0.0;
    // cmd_roll_pitch_.pitch = optimal_solution.pitch_[0];

    // if(forward_leaning_){
    //     cmd_roll_pitch_.pitch = last_pitch_input_ + 0.1*M_PI/180.0;
    //     if(cmd_roll_pitch_.pitch > pitch_max_) forward_leaning_ = false;
    // }
    // else{
    //     cmd_roll_pitch_.pitch = last_pitch_input_ - 0.1*M_PI/180.0;
    //     if(cmd_roll_pitch_.pitch < pitch_min_) forward_leaning_ = true;
    // }
    // last_pitch_input_ = cmd_roll_pitch_.pitch;
    // optimal_solution.pitch_[0] = cmd_roll_pitch_.pitch;
    
    // cmd_roll_pitch_.pitch = 0.0;

    // steering_l_.data = 0.0;
    // steering_r_.data = 0.0;
    // std::cout << "command pitch: " << cmd_roll_pitch_.pitch*180/M_PI << std::endl;

    pub_left_steering_.publish(steering_l_);
    pub_right_steering_.publish(steering_r_);
    pub_roll_pitch_.publish(cmd_roll_pitch_);

    
}

void FullBodyMPPI::publish_CandidatePath()
{
    candidate_path_marker_.markers.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++)
    {
        candidate_path_marker_.markers[i].header.frame_id = WORLD_FRAME;
        candidate_path_marker_.markers[i].header.stamp = ros::Time::now();
        candidate_path_marker_.markers[i].ns = "candidate_path";
        candidate_path_marker_.markers[i].id = i;
        candidate_path_marker_.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        candidate_path_marker_.markers[i].action = visualization_msgs::Marker::ADD;
        candidate_path_marker_.markers[i].pose.orientation.w = 1.0;
        candidate_path_marker_.markers[i].scale.x = 0.05;
        candidate_path_marker_.markers[i].color.a = 1.0;
        candidate_path_marker_.markers[i].color.r = 0.0;
        candidate_path_marker_.markers[i].color.g = 1.0;
        candidate_path_marker_.markers[i].color.b = 0.0;
        candidate_path_marker_.markers[i].lifetime = ros::Duration();
        candidate_path_marker_.markers[i].points.resize(horizon_);
        for (int t = 0; t < horizon_; t++)
        {
            geometry_msgs::Point p;
            p.x = sample[i].x_[t];
            p.y = sample[i].y_[t];
            p.z = 0.0;
            candidate_path_marker_.markers[i].points[t] = p;
        }
    }
    pub_candidate_path_.publish(candidate_path_marker_);
}

void FullBodyMPPI::determine_OptimalSolution()
{
    // Determine optimal solution
    for(int t=0; t<horizon_; t++)
    {
        optimal_solution.v_[t] = 0.0;
        optimal_solution.w_[t] = 0.0;
        optimal_solution.steer_[t] = 0.0;
        optimal_solution.pitch_[t] = 0.0;
        for(int i=0; i<num_samples_; i++)
        {
            optimal_solution.v_[t] += weights_[i] * sample[i].v_[t];
            optimal_solution.w_[t] += weights_[i] * sample[i].w_[t];
            optimal_solution.steer_[t] += weights_[i] * sample[i].steer_[t];
            optimal_solution.pitch_[t] += weights_[i] * sample[i].pitch_[t];
        }
    }

    // std::cout << "====================" << std::endl;
    // std::cout << "v: " << optimal_solution.v_[0] << " w: " << optimal_solution.w_[0] 
    // << " steer: " << optimal_solution.steer_[0]*180/M_PI << std::endl;

    // publish_OptimalPath();
}

int FullBodyMPPI::get_CurrentIndex()
{
    int index = 0;
    double min_distance = 100.0;
    for (int i = 0; i < path_.poses.size(); i++)
    {
        double distance = sqrt(pow(current_state_.x_[0] - path_.poses[i].pose.position.x, 2) + pow(current_state_.y_[0] - path_.poses[i].pose.position.y, 2));
        if (distance < min_distance)
        {
            min_distance = distance;
            index = i;
        }
    }
    return index;
}

void FullBodyMPPI::publish_RefPath()
{
    ref_path_.header.frame_id = WORLD_FRAME;
    ref_path_.header.stamp = ros::Time::now();
    ref_path_.poses.resize(horizon_);
    for(int i=0; i<horizon_; i++)
    {
        ref_path_.poses[i].pose.position.x = x_ref_[i];
        ref_path_.poses[i].pose.position.y = y_ref_[i];
        ref_path_.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw_ref_[i]);
    }
    pub_ref_path_.publish(ref_path_);
}

void FullBodyMPPI::calc_RefPath()
{
    current_index_ = get_CurrentIndex();

    double step = v_ref_ * dt_ / resolution_;
    for(int i=0; i<horizon_; i++)
    {
        int index = current_index_ + i * step;
        if(index < path_.poses.size())
        {
            x_ref_[i] = path_.poses[index].pose.position.x;
            y_ref_[i] = path_.poses[index].pose.position.y;
        }
        else
        {
            x_ref_[i] = path_.poses[path_.poses.size()-1].pose.position.x;
            y_ref_[i] = path_.poses[path_.poses.size()-1].pose.position.y;
        }
    }

    for(int i=0; i<horizon_-1; i++)
    {
        yaw_ref_[i] = atan2(y_ref_[i+1]-y_ref_[i], x_ref_[i+1]-x_ref_[i]);
    }
    
    // Publish reference path
    publish_RefPath();
}

double FullBodyMPPI::calc_MinDistance(double x, double y, std::vector<double> x_ref, std::vector<double> y_ref)
{
    double min_distance = 100.0;
    for(int i=0; i<horizon_; i++)
    {
        double distance = sqrt(pow(x - x_ref[i], 2) + pow(y - y_ref[i], 2));
        if(distance < min_distance) min_distance = distance;
    }
    return min_distance;
}
double FullBodyMPPI::calc_Cost(DecisionVariables sample)
{
    double cost = 0.0;

    for(int t=0; t < horizon_; t++)
    {
        double distance = calc_MinDistance(sample.x_[t], sample.y_[t], x_ref_, y_ref_);
        double v_cost = std::abs(v_ref_ - sample.v_[t]);

        cost += path_weight_* distance + v_weight_*(v_cost);
        // double zmp_cost = zmp_weight_ * abs(sample.zmp_x[t]);
        // cost += zmp_cost;
    }
    return cost;
}

void FullBodyMPPI::calc_Weights()
{
    calc_RefPath();
    double sum = 0.0;
    for(int i=0; i<num_samples_; i++)
    {
        double cost = calc_Cost(sample[i]);
        weights_[i] = exp(-cost / lambda_);
        sum += weights_[i];
    }
    // std::cout << "sum: " << sum << std::endl;
    for(int i=0; i<num_samples_; i++) weights_[i] /= sum;
}

void FullBodyMPPI::predict_NextState(DecisionVariables &sample, int t)
{
    double drive_accel, last_v;
    if(t == 0) last_v = odom_.twist.twist.linear.x;
    else last_v = sample.v_[t-1];
    drive_accel = (sample.v_[t] - last_v) / dt_;

    sample.x_[t+1] = sample.x_[t] + sample.v_[t] * cos(sample.yaw_[t] + sample.steer_[t]) * dt_;
    sample.y_[t+1] = sample.y_[t] + sample.v_[t] * sin(sample.yaw_[t] + sample.steer_[t]) * dt_;
    sample.yaw_[t+1] = sample.yaw_[t] + sample.w_[t] * dt_;
    sample.zmp_x[t+1] = calc_ZMP(sample.pitch_[t], drive_accel);
}

void FullBodyMPPI::predict_States()
{
    for(int i=0; i < num_samples_; i++)
    {
        // t=0の状態．現在の状態を代入
        sample[i].x_[0] = current_state_.x_[0];
        sample[i].y_[0] = current_state_.y_[0];
        sample[i].yaw_[0] = current_state_.yaw_[0];
        sample[i].zmp_x[0] = current_state_.zmp_x[0];
        //t=1以降の状態を計算
        for(int t=0; t<horizon_-1; t++){
            predict_NextState(sample[i], t);
        }
    }
    publish_CandidatePath();
}

void FullBodyMPPI::sampling()
{
    std::random_device rnd;
    std::mt19937 mt(rnd());

    for(int t=0; t < horizon_-1; t++)
    {
        std::normal_distribution<> norm_v(optimal_solution.v_[t], control_noise_);
        std::normal_distribution<> norm_w(optimal_solution.w_[t], control_noise_);
        std::normal_distribution<> norm_steer(optimal_solution.steer_[t], control_noise_);
        std::normal_distribution<> norm_roll(optimal_solution.roll_[t], control_noise_);
        std::normal_distribution<> norm_pitch(optimal_solution.pitch_[t], control_noise_);

        for(int i=0; i < num_samples_; i++)
        {
            sample[i].v_[t] = norm_v(mt);
            sample[i].w_[t] = norm_w(mt);
            sample[i].steer_[t] = norm_steer(mt);
            sample[i].roll_[t] = norm_roll(mt);
            sample[i].pitch_[t] = norm_pitch(mt);
            clamp(sample[i].v_[t], v_min_, v_max_);
            clamp(sample[i].w_[t], w_min_, w_max_);
            clamp(sample[i].steer_[t], steer_min_, steer_max_);
            clamp(sample[i].roll_[t], roll_min_, roll_max_);
            clamp(sample[i].pitch_[t], pitch_min_, pitch_max_);
        }
    }
}

void FullBodyMPPI::clamp(double &val, double min, double max)
{
    if(val < min) val = min;
    else if(val > max) val = max;
}

double FullBodyMPPI::calc_ZMP(double lean_angle, double accel)
{
    double zmp;
    zmp =l_center_of_mass_*sin(lean_angle) - accel*(l_center_of_mass_*cos(lean_angle) + h_base)/g;
    return zmp;
}

void FullBodyMPPI::get_CurrentState(double current_drive_accel_)
{
    try
    {
        listener_.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), transform_);
        tf::transformStampedTFToMsg(transform_, transform_msg_);
        current_state_.x_[0] = transform_msg_.transform.translation.x;
        current_state_.y_[0] = transform_msg_.transform.translation.y;
        current_state_.yaw_[0] = tf::getYaw(transform_msg_.transform.rotation);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    double ac = odom_.twist.twist.linear.x * odom_.twist.twist.angular.z; //向心加速度(速度方向の加速度と直交)
    double ax = current_drive_accel_ * cos(optimal_solution.steer_[0]) - ac*sin(optimal_solution.steer_[0]); //ロボット座標系x軸(worldのyaw)方向の加速度
    double ay = current_drive_accel_ * sin(optimal_solution.steer_[0]) + ac*cos(optimal_solution.steer_[0]); //ロボット座標系y軸(yaw+pi/2)方向の加速度   
    current_state_.zmp_x[0] = calc_ZMP(optimal_solution.pitch_[0], ax);  //一旦pitchだけで試す
    std::cout << "===========================================" << std::endl;
    std::cout << "zmp_x: " << current_state_.zmp_x[0] << "     ax: " << ax << std::endl;
    // current_state_.zmp_y[0] = calc_ZMP(last_roll_input_, ay);
    optimal_solution.pitch_[0] = ax *M_PI/180.0;
    // std::cout << "pitch: " << optimal_solution.pitch_[0] << std::endl;
}

void FullBodyMPPI::save_Data()
{
    
}

void FullBodyMPPI::run()
{
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        if(path_received_ && odom_received_)
        {
            if(first_roop_)
            {
                last_time_ = ros::Time::now().toSec();
                last_v_ = odom_.twist.twist.linear.x;
                first_roop_ = false;
            }
            else{
                current_time_ = ros::Time::now().toSec();
                current_v = odom_.twist.twist.linear.x;
                dt_ = current_time_ - last_time_;
                double current_drive_accel_ = (current_v - last_v_) / dt_; //速度(yaw+steer)方向の加速度
                last_time_ = current_time_;
                last_v_ = current_v;

                // 0. Get transform
                get_CurrentState(current_drive_accel_);
                // 1. Sampling
                sampling();
                // 2. Predict state
                predict_States();
                // 3. Calculate weights
                calc_Weights();
                // 4. Determine optimal solution
                determine_OptimalSolution();
                // 5. Publish cmd_pos and cmd_vel
                publish_CmdVel();
                publish_CmdPos();
                //for analysis
                save_Data();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "full_body_mppi");
    FullBodyMPPI full_body_mppi;
    full_body_mppi.run();
    return 0;
}