#include "ccv_mppi_path_tracker/full_body_mppi.h"

FullBodyMPPI::FullBodyMPPI()
    : nh_("~"), 
    tread_(0.501), wheel_radius_(0.1435),base2CoM(0.5735), ground2base(0.10), buffer_size_(5)
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
    nh_.param("path_weight", path_weight_, 1.0);
    nh_.param("v_weight", v_weight_, 1.0);
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
    sub_imu_ = nh_.subscribe("/gazebo/imu/data", 1, &FullBodyMPPI::imuCallback, this);
    // for debug
    pub_ref_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/ref_path", 1);
    pub_candidate_path_ = nh_.advertise<visualization_msgs::MarkerArray>("/ccv_mppi_path_tracker/candidate_path", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/optimal_path", 1);
    pub_zmp_y_ = nh_.advertise<std_msgs::Float64>("/ccv_mppi_path_tracker/zmp_y", 1);
    pub_drive_accel_ = nh_.advertise<std_msgs::Float64>("/ccv_mppi_path_tracker/drive_accel", 1);
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
}

void FullBodyMPPI::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    link_states_ = *msg;
    if(!link_states_received_) link_states_received_ = true;

    // tf::Matrix3x3 m(tf::Quaternion(link_states_.pose[5].orientation.x, link_states_.pose[5].orientation.y, link_states_.pose[5].orientation.z, link_states_.pose[5].orientation.w));
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // double com_x = (CoM_height * sin(pitch));
    // double com_y = (CoM_height * sin(roll));
}

void FullBodyMPPI::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(imu_).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
    if(!imu_received_) imu_received_ = true;
    double imu_accell = msg->linear_acceleration.x;
}
void FullBodyMPPI::publish_CmdVel()
{
    cmd_vel_.linear.x = optimal_solution_.v_[0];
    cmd_vel_.angular.z = optimal_solution_.w_[0];

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
    cmd_pos_.roll += optimal_solution_.roll_v_[0] * dt_;
    if (cmd_pos_.roll > roll_max_) cmd_pos_.roll = roll_max_;
    else if (cmd_pos_.roll < roll_min_) cmd_pos_.roll = roll_min_;
    
    // cmd_pos_.fore = - optimal_solution_.pitch_[0];
    // cmd_pos_.rear = optimal_solution_.pitch_[0];
    // cmd_pos_.roll = 0.0;
    // cmd_pos_.roll = -M_PI/48;
    cmd_pos_.fore = 0.0;
    cmd_pos_.rear = 0.0;

    

    std::cout << "command roll: " << cmd_pos_.roll*180/M_PI << std::endl;
    if(cmd_pos_.roll > 0.0) std::cout << "+++++++++++++++++++++++++++++++++++" << std::endl;
    else if(cmd_pos_.roll < 0.0) std::cout << "----------------------------------" << std::endl;
    
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
    
    for(int t=0; t < horizon_-1; t++)
    {
        cost += path_weight_ * calc_MinDistance(sample.x_[t], sample.y_[t], x_ref_, y_ref_);
        cost += v_weight_ * std::abs(v_ref_ - sample.v_[t]);
        cost += zmp_weight_ * abs(sample.zmp_y_[t]);
        // cost += abs(sample.roll_v_[t] - sample.roll_v_[t-1]);
        // cost += abs(sample.pitch_v_[t] * 180/M_PI);
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
    // std::cout << "ave cost: " << sum_cost / num_samples_ / (horizon_-1) << std::endl;
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

    double ac = sample.v_[t] * sample.w_[t]; //向心加速度(速度方向の加速度と直交)
    double drive_accel_x = drive_accel * cos(sample.direction_[t]) - ac*sin(sample.direction_[t]); //ロボット座標系x軸(worldのyaw)方向の加速度
    double drive_accel_y = drive_accel * sin(sample.direction_[t]) + ac*cos(sample.direction_[t]); //ロボット座標系y軸(yaw+pi/2)方向の加速度   


    double com_x = base2CoM * sin(sample.pitch_[t]); 
    double com_y = -base2CoM * sin(sample.roll_[t]); 
    double com_z = CoM_height* cos(sample.pitch_[t]) * cos(sample.roll_[t]);

    // sample.zmp_x_[t] = calc_ZMP(com_accel_x, com_z, com_x);
    sample.zmp_y_[t] = calc_ZMP(drive_accel_y, roll_accel, com_y, com_z);
    
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

double FullBodyMPPI::calc_ZMP(double drive_accel, double angular_accel, double CoM, double z)
{
    double L = 2*CoM_height;
    return CoM - (drive_accel * z / g) + ((L*L/3)+ R*R/4) *angular_accel/ g;
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
    
    // Get curretn direction
    double steer_l = joint_state_.position[5];
    double steer_r = joint_state_.position[7];
    current_state_.direction_[0] = (steer_l + steer_r) / 2.0;//大雑把

    // Get current roll rollはjointそのままでOK
    // current_state_.roll_[0] = joint_state_.position[2]; //許容範囲
    current_state_.roll_[0] = imu_roll_;
    current_state_.pitch_[0] = imu_pitch_;

    //移動による重心の加速度
    double ac = odom_.twist.twist.linear.x * odom_.twist.twist.angular.z; //向心加速度(速度方向の加速度と直交)
    double drive_accel_x = drive_accel_ * cos(current_state_.direction_[0]) - ac*sin(current_state_.direction_[0]); //ロボット座標系x軸(worldのyaw)方向の加速度
    double drive_accel_y = drive_accel_ * sin(current_state_.direction_[0]) + ac*cos(current_state_.direction_[0]); //ロボット座標系y軸(yaw+pi/2)方向の加速度   

    CoM_x = (CoM_height * sin(imu_pitch_));
    CoM_y = -(CoM_height * sin(imu_roll_));
    CoM_z = (CoM_height * cos(imu_pitch_) * cos(imu_roll_));

    current_state_.zmp_y_[0] = calc_ZMP(drive_accel_y, roll_accel_, CoM_y, CoM_z);
    std::cout << "====================================" << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "zmp_y: " << current_state_.zmp_y_[0]*100 << std::endl;
    // if(current_state_.zmp_y_[0] > 0.0) std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

}

void FullBodyMPPI::run()
{
    ros::Rate loop_rate(10);
    double last_time_, last_v_, last_roll_v_, last_pitch_v_; 

    while(ros::ok())
    {    
        if(path_received_ && odom_received_ && imu_received_ && joint_state_received_)
        {
            if(first_roop_)
            {
                last_time_ = ros::Time::now().toSec();
                last_v_ = odom_.twist.twist.linear.x;
                last_roll_v_ = joint_state_.velocity[2];
                first_roop_ = false;
            }
            else{
                dt_ = ros::Time::now().toSec() - last_time_;
                last_time_ = ros::Time::now().toSec();
                double drive_accel = (odom_.twist.twist.linear.x - last_v_) / dt_;
                double roll_accel = (joint_state_.velocity[2] - last_roll_v_) / dt_;
                last_v_ = odom_.twist.twist.linear.x;
                last_roll_v_ = joint_state_.velocity[2];

                // 0. Get Current State
                get_CurrentState(drive_accel, roll_accel, 0.0);
                // ===================================================================
                // zmp_y_.data = current_state_.zmp_y_[0];
                // pub_zmp_y_.publish(zmp_y_);
                // drive_accel_.data = drive_accel;
                // pub_drive_accel_.publish(drive_accel_);
                // ===================================================================
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