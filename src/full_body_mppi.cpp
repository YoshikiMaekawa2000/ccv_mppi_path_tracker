#include "ccv_mppi_path_tracker/full_body_mppi.h"

FullBodyMPPI::FullBodyMPPI()
    : nh_("~"), path_received_(false), odom_received_(false), joint_state_received_(false), 
    tread_(0.501), wheel_radius_(0.1435),
    weight_(60), base2CoM(0.5735), ground2base(0.10), buffer_size_(5), first_roop_(true)
{
    nh_.param("dt", dt_, 0.1);
    nh_.param("horizon", horizon_, 15);
    nh_.param("num_samples", num_samples_, 1000);
    nh_.param("control_noise", control_noise_, 0.5);
    nh_.param("lambda", lambda_, 1.0);
    nh_.param("v_max", v_max_, 3.0);
    nh_.param("w_max", w_max_, 1.0);
    nh_.param("steer_max", steer_max_, {30.0*M_PI/180.0});
    nh_.param("roll_max", roll_max_, {30.0*M_PI/180.0});
    nh_.param("pitch_max", pitch_max_, {15.0*M_PI/180.0});
    nh_.param("roll_v_max", roll_v_max_, {30.0*M_PI/180.0});
    nh_.param("pitch_v_max", pitch_v_max_, {15.0*M_PI/180.0});
    nh_.param("v_min", v_min_, -3.0);
    nh_.param("w_min", w_min_, -1.0);
    nh_.param("steer_min", steer_min_, {-30.0*M_PI/180.0});
    nh_.param("roll_min", roll_min_, {-30.0*M_PI/180.0});
    nh_.param("pitch_min", pitch_min_, {-15.0*M_PI/180.0});
    nh_.param("roll_v_min", roll_v_min_, {-30.0*M_PI/180.0});
    nh_.param("pitch_v_min", pitch_v_min_, {-15.0*M_PI/180.0});
    nh_.param("pitch_offset", pitch_offset_, {0.0*M_PI/180.0});
    nh_.param("v_ref", v_ref_, 1.2);
    nh_.param("resolution", resolution_, 0.1);
    nh_.param("exploration_noise", exploration_noise_, 0.1);
    nh_.param("world_frame", WORLD_FRAME, std::string("odom"));
    nh_.param("robot_frame", ROBOT_FRAME, std::string("base_link"));
    nh_.param("CoM_frame", CoM_FRAME, std::string("battery_box_link"));
    nh_.param("path_weight", path_weight_, 1.0);
    nh_.param("v_weight", v_weight_, 100.0);
    nh_.param("zmp_weight", zmp_weight_, 1.0);
    
    sample.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++) sample[i] = RobotStates(horizon_);
    optimal_solution_ = RobotStates(horizon_);
    current_state_ = RobotStates(1);

    x_ref_.resize(horizon_);
    y_ref_.resize(horizon_);
    yaw_ref_.resize(horizon_);
    weights_.resize(num_samples_);

     // publishers
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);
    // subscribers
    sub_path_ = nh_.subscribe("/reference_path", 1, &FullBodyMPPI::pathCallback, this);
    sub_odom_ = nh_.subscribe("/odom", 1, &FullBodyMPPI::odomCallback, this);
    sub_joint_state_ = nh_.subscribe("/sq2_ccv/joint_states", 1, &FullBodyMPPI::jointStateCallback, this);
    sub_link_states_ = nh_.subscribe("/gazebo/link_states", 1, &FullBodyMPPI::linkStatesCallback, this);
    // for debug
    pub_ref_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/ref_path", 1);
    pub_candidate_path_ = nh_.advertise<visualization_msgs::MarkerArray>("/ccv_mppi_path_tracker/candidate_path", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/optimal_path", 1);
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
    // std::cout << "====================" << std::endl;
    // std::cout << "accel x: " << odom_.twist.twist.linear.x << std::endl;
}

void FullBodyMPPI::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state_ = *msg;
    if(!joint_state_received_) joint_state_received_ = true;

    // current_state_.pitch_[0] = pitch;
    // current_state_.pitch_[0] = ((abs(front_caster_pitch) + abs(back_caster_pitch))/2.0) * (forward_leaning_ ? 1 : -1);
    // current_state_.roll_[0] = joint_state_.position[2];

    // std::cout << "====================" << std::endl;
    // std::cout << "front casters pitch: " << joint_state_.position[3]*180/M_PI << std::endl;
    // std::cout << "back casters pitch: " << joint_state_.position[0]*180/M_PI << std::endl;
    // std::cout << "roll: " << joint_state_.position[2]*180/M_PI << std::endl;
    // std::cout << "roll vel: " << joint_state_.velocity[2]*180/M_PI << std::endl;
    // std::cout << "steer_l: " << joint_state_.position[5]*180/M_PI << std::endl;
    // std::cout << "steer_r: " << joint_state_.position[7]*180/M_PI << std::endl;
}

void FullBodyMPPI::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    link_states_ = *msg;
    if(!link_states_received_) link_states_received_ = true;
    // std::cout << link_states_.name[4] << std::endl;
    tf::Matrix3x3 m(tf::Quaternion(link_states_.pose[4].orientation.x, link_states_.pose[4].orientation.y, link_states_.pose[4].orientation.z, link_states_.pose[4].orientation.w));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // std::cout << "roll: " << roll*180/M_PI << " pitch: " << pitch*180/M_PI << " yaw: " << yaw*180/M_PI << std::endl;
    // std::cout << "gazebo  pitch: " << pitch*180/M_PI << std::endl;

    // double x = link_states_.pose[4].position.x - link_states_.pose[1].position.x;
    // double y = link_states_.pose[4].position.y - link_states_.pose[1].position.y;
    // double z = link_states_.pose[4].position.z - link_states_.pose[1].position.z;

    // std::cout << "====================" << std::endl;
    // std::cout << "name: " << link_states_.name[4] << std::endl;
    // std::cout << "battery box pose: " << link_states_.pose[4].position.x << " " << link_states_.pose[4].position.y << " " << link_states_.pose[4].position.z << std::endl;
    
}
void FullBodyMPPI::publish_CmdVel()
{
    // std::cout << "v: " << optimal_solution.v_[0] << " w: " << optimal_solution.w_[0] << std::endl;

    cmd_vel_.linear.x = optimal_solution_.v_[0];
    cmd_vel_.angular.z = optimal_solution_.w_[0];
    // cmd_vel_.linear.x = 3.0;
    // cmd_vel_.angular.z = 0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FullBodyMPPI::publish_CmdPos()
{
    double R = fabs(optimal_solution_.v_[0] / optimal_solution_.w_[0]);
    double steer_in = std::atan2(R*sin(optimal_solution_.direction_[0]), R*cos(optimal_solution_.direction_[0]) - tread_/2.0);
    double steer_out = std::atan2(R*sin(optimal_solution_.direction_[0]), R*cos(optimal_solution_.direction_[0]) + tread_/2.0);

    if(optimal_solution_.w_[0] > 0.0){
        cmd_pos_.steer_l = steer_in;
        cmd_pos_.steer_r = steer_out;
    }
    else{
        cmd_pos_.steer_l = steer_out;
        cmd_pos_.steer_r = steer_in;
    }
    // std::cout << "steer_l: " << steering_l_.data*180/M_PI << " steer_r: " << steering_r_.data*180/M_PI << std::endl;

    cmd_pos_.roll = current_state_.roll_[0] + optimal_solution_.roll_v_[0] * dt_;
    if (cmd_pos_.roll > roll_max_) cmd_pos_.roll = roll_max_;
    else if (cmd_pos_.roll < roll_min_) cmd_pos_.roll = roll_min_;
    
    // cmd_pos_.fore = - optimal_solution_.pitch_[0];
    // cmd_pos_.rear = optimal_solution_.pitch_[0];
    // cmd_pos_.roll = 0.0;
    // cmd_pos_.roll = -M_PI/48;
    cmd_pos_.fore = 0.0;
    cmd_pos_.rear = 0.0;

    

    // std::cout << "command pitch: " << optimal_solution_.pitch_[0]*180/M_PI << std::endl;
    std::cout << "command roll: " << cmd_pos_.roll *180/M_PI << std::endl;
    std::cout << "====================" << std::endl;
    pub_cmd_pos_.publish(cmd_pos_);
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
            // p.x = sample[i].x_[t];
            // p.y = sample[i].y_[t];
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
        optimal_solution_.v_[t] = 0.0;
        optimal_solution_.w_[t] = 0.0;
        optimal_solution_.direction_[t] = 0.0;
        optimal_solution_.roll_v_[t] = 0.0;
        optimal_solution_.pitch_v_[t] = 0.0;
        for(int i=0; i<num_samples_; i++)
        {
            optimal_solution_.v_[t] += weights_[i] * sample[i].v_[t];
            optimal_solution_.w_[t] += weights_[i] * sample[i].w_[t];
            optimal_solution_.direction_[t] += weights_[i] * sample[i].direction_[t];
            optimal_solution_.roll_v_[t] += weights_[i] * sample[i].roll_v_[t];
            optimal_solution_.pitch_v_[t] += weights_[i] * sample[i].pitch_v_[t];
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
double FullBodyMPPI::calc_Cost(RobotStates sample)
{
    double cost = 0.0;
    
    for(int t=0; t < horizon_; t++)
    {
        cost += path_weight_ * calc_MinDistance(sample.x_[t], sample.y_[t], x_ref_, y_ref_);
        cost += v_weight_ * std::abs(v_ref_ - sample.v_[t]);
        cost += zmp_weight_ * abs(sample.zmp_y_[t]);
        // if(t != horizon_ -1 ) cost += abs(sample.roll_v_[t]);
        // cost += zmp_weight_ * abs(sample.zmp_x_[t]);
    }
    return cost;
}

void FullBodyMPPI::calc_Weights()
{
    calc_RefPath();
    double sum = 0.0;
    double sum_cost = 0.0;
    for(int i=0; i<num_samples_; i++)
    {
        double cost = calc_Cost(sample[i]);
        sum_cost += cost;
        weights_[i] = exp(-cost / lambda_);
        sum += weights_[i];
    }
    std::cout << "ave cost: " << sum_cost / num_samples_ / horizon_ << std::endl;
    for(int i=0; i<num_samples_; i++) weights_[i] /= sum;
}

void FullBodyMPPI::predict_NextState(RobotStates &sample, int t)
{
    sample.x_[t] = sample.x_[t-1] + sample.v_[t-1] * cos(sample.yaw_[t-1] + sample.direction_[t-1]) * dt_;
    sample.y_[t] = sample.y_[t-1] + sample.v_[t-1] * sin(sample.yaw_[t-1] + sample.direction_[t-1]) * dt_;
    sample.yaw_[t] = sample.yaw_[t-1] + sample.w_[t-1] * dt_;

    double drive_accel, roll_accel, pitch_accel;

    drive_accel = (sample.v_[t] - sample.v_[t-1]) / dt_;
    roll_accel = (sample.roll_v_[t] - sample.roll_v_[t-1]) / dt_;
    pitch_accel = (sample.pitch_v_[t] - sample.pitch_v_[t-1]) / dt_;

    sample.roll_[t] = sample.roll_[t-1] + sample.roll_v_[t-1] * dt_;
    sample.pitch_[t] = sample.pitch_[t-1] + sample.pitch_v_[t-1] * dt_;
    clamp(sample.roll_[t], roll_min_, roll_max_);
    clamp(sample.pitch_[t], pitch_min_, pitch_max_);

    double ac = sample.v_[t-1] * sample.w_[t-1]; //向心加速度(速度方向の加速度と直交)
    double ax = drive_accel * cos(sample.direction_[t-1]) - ac*sin(sample.direction_[t-1]); //ロボット座標系x軸(worldのyaw)方向の加速度
    double ay = drive_accel * sin(sample.direction_[t-1]) + ac*cos(sample.direction_[t-1]); //ロボット座標系y軸(yaw+pi/2)方向の加速度   

    double com_ax = ax + pitch_accel * base2CoM;
    double com_ay = ay + roll_accel * base2CoM;
    double com_x = base2CoM * sin(sample.pitch_[t]); //大雑把
    double com_y = -base2CoM * sin(sample.roll_[t]); //大雑把
    double com_z = ground2base + base2CoM * cos(sample.pitch_[t]) * cos(sample.roll_[t]); //大雑把

    sample.zmp_x_[t] = calc_ZMP(com_ax, com_z, com_x);
    sample.zmp_y_[t] = calc_ZMP(com_ay, com_z, com_y);
    
}

void FullBodyMPPI::predict_States()
{
    for(int i=0; i < num_samples_; i++)
    {
        sample[i].x_[0] = current_state_.x_[0];
        sample[i].y_[0] = current_state_.y_[0];
        sample[i].yaw_[0] = current_state_.yaw_[0];
        sample[i].zmp_x_[0] = current_state_.zmp_x_[0];
        sample[i].zmp_y_[0] = current_state_.zmp_y_[0];
        sample[i].roll_[0] = current_state_.roll_[0];
        sample[i].pitch_[0] = current_state_.pitch_[0];
        for(int t=1; t<horizon_-1; t++){
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
        std::normal_distribution<> norm_v(optimal_solution_.v_[t], control_noise_);
        std::normal_distribution<> norm_w(optimal_solution_.w_[t], control_noise_);
        std::normal_distribution<> norm_direction(optimal_solution_.direction_[t], control_noise_);
        std::normal_distribution<> norm_roll_v(optimal_solution_.roll_v_[t], control_noise_);
        std::normal_distribution<> norm_pitch_v(optimal_solution_.pitch_v_[t], control_noise_);

        for(int i=0; i < num_samples_; i++)
        {
            sample[i].v_[t] = norm_v(mt);
            sample[i].w_[t] = norm_w(mt);
            sample[i].direction_[t] = norm_direction(mt);
            sample[i].roll_v_[t] = norm_roll_v(mt);
            sample[i].pitch_v_[t] = norm_pitch_v(mt);
            clamp(sample[i].v_[t], v_min_, v_max_);
            clamp(sample[i].w_[t], w_min_, w_max_);
            clamp(sample[i].direction_[t], steer_min_, steer_max_);
            clamp(sample[i].roll_v_[t], roll_v_min_, roll_v_max_);
            clamp(sample[i].pitch_v_[t], pitch_v_min_, pitch_v_max_);
        }
    }
}

void FullBodyMPPI::clamp(double &val, double min, double max)
{
    if(val < min) val = min;
    else if(val > max) val = max;
}

double FullBodyMPPI::calc_ZMP(double accel, double CoM_z, double CoM_coord)
{
    return -accel * CoM_z / g + CoM_coord;
}

void FullBodyMPPI::get_CurrentState(double drive_accel_, double roll_accel_, double pitch_accel_)
{
    // Get current x, y, yaw
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

    // Get current pitch ここはあとで関数化
    double front_caster_pitch = joint_state_.position[3];
    double back_caster_pitch = joint_state_.position[0];
    if(front_caster_pitch < 0.0 && back_caster_pitch > 0.0) forward_leaning_ = true;
    else if(front_caster_pitch > 0.0 && back_caster_pitch < 0.0) forward_leaning_ = false;
    current_state_.pitch_[0] = ((abs(front_caster_pitch) + abs(back_caster_pitch))/4.0) * (forward_leaning_ ? 1 : -1); //かなり大雑把

    // Get curretn direction
    double steer_l = joint_state_.position[5];
    double steer_r = joint_state_.position[7];
    current_state_.direction_[0] = (steer_l + steer_r) / 2.0;//大雑把

    // Get current roll rollはjointそのままでOK
    current_state_.roll_[0] = joint_state_.position[2]; //許容範囲
    
    // 重心の座標．大雑把
    double com_x = base2CoM * sin(current_state_.pitch_[0]); //大雑把
    double com_y = -base2CoM * sin(current_state_.roll_[0]); //大雑把
    double com_z = ground2base + base2CoM * cos(current_state_.pitch_[0]) * cos(current_state_.roll_[0]); //大雑把

    //移動による重心の加速度
    double ac = odom_.twist.twist.linear.x * odom_.twist.twist.angular.z; //向心加速度(速度方向の加速度と直交)
    double drive_accel_x = drive_accel_ * cos(current_state_.direction_[0]) - ac*sin(current_state_.direction_[0]); //ロボット座標系x軸(worldのyaw)方向の加速度
    double drive_accel_y = drive_accel_ * sin(current_state_.direction_[0]) + ac*cos(current_state_.direction_[0]); //ロボット座標系y軸(yaw+pi/2)方向の加速度   

    //回転によって発生する加速度を追加
    double com_accel_x = drive_accel_x + pitch_accel_ * base2CoM;
    double com_accel_y = drive_accel_x + roll_accel_ * base2CoM;

    current_state_.zmp_x_[0] = calc_ZMP(com_accel_x, com_z, com_x);
    current_state_.zmp_y_[0] = calc_ZMP(com_accel_y, com_z, com_y);
    // std::cout << "current zmp_x: " << current_state_.zmp_x_[0]*100 << std::endl;
    std::cout << "current zmp_y: " << current_state_.zmp_y_[0]*100 << std::endl;
    std::cout << "current roll: " << current_state_.roll_[0]*180/M_PI << std::endl;
}

void FullBodyMPPI::save_Data()
{
    
}

void FullBodyMPPI::run()
{
    ros::Rate loop_rate(10);
    double last_time_, last_v_, last_roll_v_, last_pitch_v_; 

    while(ros::ok())
    {    
        if(path_received_ && odom_received_)
        {
            if(first_roop_)
            {
                last_time_ = ros::Time::now().toSec();
                last_v_ = odom_.twist.twist.linear.x;
                last_roll_v_ = joint_state_.velocity[2];
                // last_pitch_v_ = joint_state_.velocity[3];
                
                first_roop_ = false;
            }
            else{
                // avgDriveAccel.addVelocity(ros::Time::now().toSec(), odom_.twist.twist.linear.x);
                dt_ = ros::Time::now().toSec() - last_time_;
                last_time_ = ros::Time::now().toSec();
                double drive_accel = (odom_.twist.twist.linear.x - last_v_) / dt_;
                double roll_accel = (joint_state_.velocity[2] - last_roll_v_) / dt_;
                // double pitch_accel = (joint_state_.velocity[3] - last_pitch_v_) / dt_;
                last_v_ = odom_.twist.twist.linear.x;
                last_roll_v_ = joint_state_.velocity[2];
                // last_pitch_v_ = joint_state_.velocity[3]; 

                // 0. Get Current State
                get_CurrentState(drive_accel, roll_accel, 0.0);
                // 1. Sampling
                sampling();
                // 2. Predict state
                predict_States();
                // std::cout << "roll v min: " << roll_v_min_*180/M_PI << " roll v max: " << roll_v_max_*180/M_PI << std::endl;

                double max_sample_zmp_y = sample[0].zmp_y_[0];
                double min_sample_zmp_y = sample[0].zmp_y_[0];
                // double max_sample_roll = sample[0].roll_[0];
                // double min_sample_roll = sample[0].roll_[0];
                // double max_sample_roll_v = sample[0].roll_v_[0];
                // double min_sample_roll_v = sample[0].roll_v_[0];
                for(int i=0; i<num_samples_; i++)
                {
                    for(int t=0; t<horizon_-1; t++)
                    {
                        if(sample[i].zmp_y_[t] > max_sample_zmp_y) max_sample_zmp_y = sample[i].zmp_y_[t];
                        if(sample[i].zmp_y_[t] < min_sample_zmp_y) min_sample_zmp_y = sample[i].zmp_y_[t];
                        // if(sample[i].roll_[t] > max_sample_roll) max_sample_roll = sample[i].roll_[t];
                        // if(sample[i].roll_[t] < min_sample_roll) min_sample_roll = sample[i].roll_[t];
                        // if(sample[i].roll_v_[t] > max_sample_roll_v) max_sample_roll_v = sample[i].roll_v_[t];
                        // if(sample[i].roll_v_[t] < min_sample_roll_v) min_sample_roll_v = sample[i].roll_v_[t];
                    }
                }
                std::cout << "sample  zmp_y:  max:" << max_sample_zmp_y*100 << " | min: " << min_sample_zmp_y*100 << std::endl;
                // std::cout << "sample  roll:  max:" << max_sample_roll*180/M_PI << " | min: " << min_sample_roll*180/M_PI << std::endl;
                // std::cout << "max roll_v: " << max_sample_roll_v*180/M_PI << " min roll_v: " << min_sample_roll_v*180/M_PI << std::endl;


                // 3. Calculate weights
                calc_Weights();
                // 4. Determine optimal solution
                determine_OptimalSolution();
                // 5. Publish cmd_pos and cmd_vel
                publish_CmdVel();
                publish_CmdPos();
                //for analysis
                // save_Data();
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