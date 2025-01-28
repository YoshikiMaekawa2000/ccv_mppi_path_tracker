#include "ccv_mppi_path_tracker/full_body_mppi.h"


FullBodyMPPI::FullBodyMPPI()
    : nh_("~"), 
    tread_(0.501), wheel_radius_(0.1435),base2CoM(0.5735), ground2base(0.10), buffer_size_(5), mass(60.0)
{
    nh_.param("dt", dt_, 0.1);
    nh_.param("horizon", horizon_, 15);
    nh_.param("num_samples", num_samples_, 10000);
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
    nh_.param("imu_frame", IMU_FRAME, std::string("imu_link"));
    nh_.param("path_weight", path_weight_, 1.0);
    nh_.param("v_weight", v_weight_, 1.0);
    nh_.param("zmp_weight", zmp_weight_, 1.0);
    nh_.param("roll_v_weight", roll_v_weight_, 1.0);
    nh_.param("off", off_, false);
    if(off_) {
        zmp_weight_ = 0.0;
        roll_v_weight_ = 0.0;
    }

    //gazeboでだけ使用
    force_sensor_topic_ = {
        "/left_force_sensor/raw",
        "/right_force_sensor/raw",
        "/front_left_force_sensor/raw",
        "/front_right_force_sensor/raw",
        "/back_left_force_sensor/raw",
        "/back_right_force_sensor/raw"
    };
    base2front_l_caster = Eigen::Vector3d(0.245, 0.167, -0.003);
    base2front_r_caster = Eigen::Vector3d(0.245, -0.167, -0.004);
    base2back_l_caster = Eigen::Vector3d(-0.245, -0.167, -0.004);
    base2back_r_caster = Eigen::Vector3d(-0.245, 0.167, -0.003);
    base2wheel_l = Eigen::Vector3d(0.0, 0.225, 0.075);
    base2wheel_r = Eigen::Vector3d(0.0, -0.225, 0.075);
    contactPositions = {base2wheel_l, base2wheel_r, base2front_l_caster, base2front_r_caster, base2back_l_caster, base2back_r_caster};

    filterd_imu_.angular_velocity.x = 0.0;
    filterd_imu_.angular_velocity.y = 0.0;
    filterd_imu_.angular_velocity.z = 0.0;
    filterd_imu_.linear_acceleration.x = 0.0;
    filterd_imu_.linear_acceleration.y = 0.0;
    filterd_imu_.linear_acceleration.z = 0.0;
    
    sample.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++) sample[i] = RobotStates(horizon_);
    optimal_solution_ = RobotStates(horizon_);
    current_state_ = RobotStates(1);
    current_state_.zmp_x_[0] = 0.0;
    current_state_.zmp_y_[0] = 0.0;

    true_ZMP = Eigen::Vector3d::Zero();

    x_ref_.resize(horizon_);
    y_ref_.resize(horizon_);
    yaw_ref_.resize(horizon_);
    weights_.resize(num_samples_);

    base2CoM = upper_body_height/2;
    Eigen::Vector3d diagonal_values(
        (mass * (upper_body_width*upper_body_width + upper_body_height*upper_body_height))/12 + mass * base2CoM*base2CoM,
        (mass * (upper_body_height*upper_body_height + upper_body_depth*upper_body_depth))/12 + mass * base2CoM*base2CoM,
        (mass * (upper_body_depth*upper_body_depth + upper_body_width*upper_body_width))/12);
    I_O = diagonal_values.asDiagonal();

     // publishers
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);
    // subscribers
    sub_path_ = nh_.subscribe("/reference_path", 1, &FullBodyMPPI::pathCallback, this);
    sub_odom_ = nh_.subscribe("/odom", 1, &FullBodyMPPI::odomCallback, this);
    sub_joint_state_ = nh_.subscribe("/sq2_ccv/joint_states", 1, &FullBodyMPPI::jointStateCallback, this);
    sub_link_states_ = nh_.subscribe("/gazebo/link_states", 1, &FullBodyMPPI::linkStatesCallback, this);
    sub_imu_ = nh_.subscribe("/gazebo/imu/data", 1, &FullBodyMPPI::imuCallback, this);
    for(const auto& topic : force_sensor_topic_){
        sub_force_sensor_.push_back(nh_.subscribe<geometry_msgs::WrenchStamped>(topic, 1, boost::bind(&FullBodyMPPI::wrenchCallback, this, _1, topic)));
    }
    
    // for debug
    pub_ref_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/ref_path", 1);
    pub_candidate_path_ = nh_.advertise<visualization_msgs::MarkerArray>("/ccv_mppi_path_tracker/candidate_path", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/optimal_path", 1);
    pub_zmp_y_ = nh_.advertise<std_msgs::Float64>("/ccv_mppi_path_tracker/zmp_y", 1);
    pub_drive_accel_ = nh_.advertise<std_msgs::Float64>("/ccv_mppi_path_tracker/drive_accel", 1);
    pub_true_zmp_ = nh_.advertise<std_msgs::Float64>("/ccv_mppi_path_tracker/true_zmp", 1);
}
void FullBodyMPPI::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, const std::string &sensor_topic)
{
    force_sensor_data_[sensor_topic] = *msg;
    try
    {
        if (sensor_topic == "/right_force_sensor/raw")
        {
            std::cout << std::fixed << std::setprecision(3);
            listener_.lookupTransform(ROBOT_FRAME, RIGHT_WHEEL, ros::Time(0), transform_);
            tf::Vector3 right_force(
                force_sensor_data_[sensor_topic].wrench.force.x,
                force_sensor_data_[sensor_topic].wrench.force.y,
                force_sensor_data_[sensor_topic].wrench.force.z);

            tf::Vector3 transformed_right_force = transform_.getBasis() * right_force;

            force_sensor_data_[sensor_topic].wrench.force.x = transformed_right_force.x();
            force_sensor_data_[sensor_topic].wrench.force.y = transformed_right_force.y();
            force_sensor_data_[sensor_topic].wrench.force.z = transformed_right_force.z();
        }

        else if (sensor_topic == "/left_force_sensor/raw")
        {
            std::cout << std::fixed << std::setprecision(3);
            listener_.lookupTransform(ROBOT_FRAME, LEFT_WHEEL, ros::Time(0), transform_);
            tf::Vector3 left_force(
                force_sensor_data_[sensor_topic].wrench.force.x,
                force_sensor_data_[sensor_topic].wrench.force.y,
                force_sensor_data_[sensor_topic].wrench.force.z);
            
            tf::Vector3 transformed_left_force = transform_.getBasis() * left_force;

            force_sensor_data_[sensor_topic].wrench.force.x = transformed_left_force.x();
            force_sensor_data_[sensor_topic].wrench.force.y = transformed_left_force.y();
            force_sensor_data_[sensor_topic].wrench.force.z = transformed_left_force.z();
        }
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("TF Error: %s", ex.what());
    }
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
    // ノイズ除去
    // filterd_imu_.angular_velocity.x = alpha * msg->angular_velocity.x + (1 - alpha) * filterd_imu_.angular_velocity.x;
    // filterd_imu_.angular_velocity.y = alpha * msg->angular_velocity.y + (1 - alpha) * filterd_imu_.angular_velocity.y;
    // filterd_imu_.angular_velocity.z = alpha * msg->angular_velocity.z + (1 - alpha) * filterd_imu_.angular_velocity.z;
    // filterd_imu_.linear_acceleration.x = alpha * msg->linear_acceleration.x + (1 - alpha) * filterd_imu_.linear_acceleration.x;
    // filterd_imu_.linear_acceleration.y = alpha * msg->linear_acceleration.y + (1 - alpha) * filterd_imu_.linear_acceleration.y;
    // filterd_imu_.linear_acceleration.z = alpha * msg->linear_acceleration.z + (1 - alpha) * filterd_imu_.linear_acceleration.z;
    filterd_imu_.angular_velocity.x = msg->angular_velocity.x;
    filterd_imu_.angular_velocity.y = msg->angular_velocity.y;
    filterd_imu_.angular_velocity.z = msg->angular_velocity.z;
    filterd_imu_.linear_acceleration.x = msg->linear_acceleration.x;
    filterd_imu_.linear_acceleration.y = msg->linear_acceleration.y;
    filterd_imu_.linear_acceleration.z = msg->linear_acceleration.z;
    // IMUリンクのworld座標での傾きを取得
    imu_orientation_ = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(imu_orientation_).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
   //座標変換 
    try{
        listener_.lookupTransform(ROBOT_FRAME, IMU_FRAME, ros::Time(0), transform_);
        tf::Vector3 accel_imu(
        filterd_imu_.linear_acceleration.x,
        filterd_imu_.linear_acceleration.y,
        filterd_imu_.linear_acceleration.z);
        accel_base = transform_.getBasis() * accel_imu;
        accel_x = accel_base.x();
        accel_y = accel_base.y();
        accel_z = accel_base.z();
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("TF Error: %s", ex.what());
    }
    //重力補償.正しいと思うコード実装しても挙動が間違ってた．これは正しくないコードだと思うけど挙動がそれっぽいからいったんこれ使う．
    accel_x -= g*sin(imu_pitch_);
    // accel_z -= g*cos(imu_roll_)*sin(imu_pitch_);
    if(!imu_received_) imu_received_ = true;
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
    direction_input = optimal_solution_.direction_[0];

    if(optimal_solution_.w_[0] > 0.0){
        cmd_pos_.steer_l = steer_in;
        cmd_pos_.steer_r = steer_out;
    }
    else{
        cmd_pos_.steer_l = steer_out;
        cmd_pos_.steer_r = steer_in;
    }

    cmd_pos_.roll = current_state_.roll_[0] + optimal_solution_.roll_v_[0] * dt_;
    if (cmd_pos_.roll > roll_max_) cmd_pos_.roll = roll_max_;
    else if (cmd_pos_.roll < roll_min_) cmd_pos_.roll = roll_min_;
    if(off_) cmd_pos_.roll = 0.0;
    cmd_pos_.fore = pitch_offset_;
    cmd_pos_.rear = pitch_offset_;
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
    

    for(int t=0; t < horizon_-2; t++)
    {
        cost += path_weight_ * calc_MinDistance(sample.x_[t], sample.y_[t], x_ref_, y_ref_);
        cost += v_weight_ * std::abs(v_ref_ - sample.v_[t]);

        cost += zmp_weight_ * abs(sample.zmp_y_[t]);
        cost += roll_v_weight_*abs(sample.roll_v_[t+1] - sample.roll_v_[t]);
    
    }
    return cost;
}

void FullBodyMPPI::calc_Weights()
{
    calc_RefPath();
    double sum = 0.0;
    double sum_cost = 0.0;
    double min_zmp_y = 100.0;
    for(int i=0; i<num_samples_; i++)
    {
        if(abs(sample[i].zmp_y_[0]) < min_zmp_y) min_zmp_y = sample[i].zmp_y_[0];
        double cost = calc_Cost(sample[i]);
        sum_cost += cost;
        weights_[i] = exp(-cost / lambda_);
        sum += weights_[i];
    }
    // std::cout << "ave cost: " << sum_cost / num_samples_ / (horizon_-1) << std::endl;
    for(int i=0; i<num_samples_; i++) weights_[i] /= sum;
    std::cout << "min zmp_y: " << min_zmp_y*100 << std::endl;
}

void FullBodyMPPI::predict_NextState(RobotStates &sample, int t)
{
    sample.x_[t+1] = sample.x_[t] + sample.v_[t] * cos(sample.yaw_[t] + sample.direction_[t]) * dt_;
    sample.y_[t+1] = sample.y_[t] + sample.v_[t] * sin(sample.yaw_[t] + sample.direction_[t]) * dt_;
    sample.yaw_[t+1] = sample.yaw_[t] + sample.w_[t] * dt_;
    sample.roll_[t+1] = sample.roll_[t] + sample.roll_v_[t] * dt_;
    sample.pitch_[t+1] = sample.pitch_[t] + sample.pitch_v_[t] * dt_;
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
        for(int t=0; t<horizon_-1; t++){
            predict_NextState(sample[i], t);
        }
        for(int t=0; t<horizon_-2; t++){
            double drive_accel = (sample[i].v_[t+1] - sample[i].v_[t]) / dt_;

            double ac = sample[i].v_[t] * sample[i].w_[t]; //向心加速度(速度方向の加速度と直交)
            double drive_accel_x = drive_accel * cos(sample[i].direction_[t]) - ac*sin(sample[i].direction_[t]); //ロボット座標系x軸(worldのyaw)方向の加速度
            double drive_accel_y = drive_accel * sin(sample[i].direction_[t]) + ac*cos(sample[i].direction_[t]); //ロボット座標系y軸(yaw+pi/2)方向の加速度   

            Eigen::Vector3d accel = Eigen::Vector3d(drive_accel_x, drive_accel_y, 0.0);
            // Eigen::Vector3d alpha = Eigen::Vector3d(roll_accel, pitch_accel, 0.0);
            Eigen::Vector3d next_omega = Eigen::Vector3d(sample[i].roll_v_[t+1], sample[i].pitch_v_[t+1], sample[i].w_[t+1]);
            Eigen::Vector3d omega = Eigen::Vector3d(sample[i].roll_v_[t], sample[i].pitch_v_[t], sample[i].w_[t]);
            Eigen::Vector3d HG_next = I_O * next_omega;
            Eigen::Vector3d HG = I_O * omega;
            Eigen::Vector3d HG_dot = (HG_next - HG) / dt_;
            Eigen::Vector3d CoM = Eigen::Vector3d(base2CoM * sin(sample[i].pitch_[t]), -base2CoM * sin(sample[i].roll_[t]), base2CoM * cos(sample[i].pitch_[t]) * cos(sample[i].roll_[t]));
            Eigen::Vector3d ZMP = computeZMPfromModel(CoM, accel, HG_dot);
            sample[i].zmp_x_[t] = ZMP.x();
            sample[i].zmp_y_[t] = ZMP.y();
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

void FullBodyMPPI::get_CurrentState()
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
    // Get current roll and pitch
    current_state_.roll_[0] = imu_roll_;
    current_state_.pitch_[0] = imu_pitch_;
    // Get current zmp
    Eigen::Vector3d CoM = Eigen::Vector3d(base2CoM * sin(imu_pitch_), -base2CoM * sin(imu_roll_), base2CoM * cos(imu_pitch_) * cos(imu_roll_));
    Eigen::Vector3d accel = Eigen::Vector3d(accel_x, accel_y, 0.0);
    Eigen::Vector3d omega = Eigen::Vector3d(filterd_imu_.angular_velocity.x, filterd_imu_.angular_velocity.y, filterd_imu_.angular_velocity.z);
    Eigen::Vector3d H_G = I_O * omega;
    Eigen::Vector3d H_Gdot = (H_G - last_HG) / dt_;
    last_HG = H_G;
    last_imu_ = filterd_imu_;
    Eigen::Vector3d ZMP = computeZMPfromModel(CoM, accel, H_Gdot);
    
    // current_state_.zmp_x_[0] = ZMP.x();
    // current_state_.zmp_y_[0] = ZMP.y();
    current_state_.zmp_x_[0] = alpha*ZMP.x() + (1-alpha)*current_state_.zmp_x_[0];
    current_state_.zmp_y_[0] = alpha*ZMP.y() + (1-alpha)*current_state_.zmp_y_[0];
}

void FullBodyMPPI::calc_true_ZMP()
{
    std::vector<Eigen::Vector3d> contactForces;

    std::cout << "====================" << std::endl;
    for(const auto& topic : force_sensor_topic_){
        contactForces.push_back(Eigen::Vector3d(force_sensor_data_[topic].wrench.force.x, force_sensor_data_[topic].wrench.force.y, force_sensor_data_[topic].wrench.force.z));
    }
    Eigen::Vector3d n(0.0, 0.0, 1.0); //床面法線ベクトル
    Eigen::Vector3d sumF=Eigen::Vector3d::Zero();
    Eigen::Vector3d sumM=Eigen::Vector3d::Zero();
    for(size_t i=0; i<contactPositions.size(); ++i){
        if(contactForces[i].z() > 0.0){
            const Eigen::Vector3d& ri = contactPositions[i];
            const Eigen::Vector3d& fi = contactForces[i];
            sumF += fi;
            sumM += ri.cross(fi);
        }
    }
    double denom=sumF.dot(n);
    if(std::fabs(denom) < 1e-6){
        std::cout << "denom is too small" << std::endl;
        return;
    }
    Eigen::Vector3d numerator = n.cross(sumM);
    // true_ZMP = numerator/denom;
    true_ZMP = alpha*(numerator/denom) + (1-alpha)*true_ZMP;
}
Eigen::Vector3d FullBodyMPPI::computeZMPfromModel(Eigen::Vector3d CoM, Eigen::Vector3d accel, Eigen::Vector3d HGdot)
{
    Eigen::Vector3d z(0.0, 0.0, 1.0);
    Eigen::Vector3d M_O =CoM.cross(mass * gravity_) - CoM.cross(mass * accel) - HGdot;
    Eigen::Vector3d ZMP = z.cross(M_O) / (mass * (gravity_ - accel).dot(z));
    return ZMP;
}

void FullBodyMPPI::run()
{
    ros::Rate loop_rate(10);
    double last_time_; 

    while(ros::ok())
    {    
        if(path_received_ && odom_received_ && imu_received_ && joint_state_received_)
        {
            if(first_roop_)
            {
                last_imu_ = filterd_imu_;
                last_time_ = ros::Time::now().toSec();
                first_roop_ = false;
            }
            else{
                dt_ = ros::Time::now().toSec() - last_time_;
                last_time_ = ros::Time::now().toSec();
                calc_true_ZMP();
                // 0. Get Current State
                get_CurrentState();
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "zmp_x: " << current_state_.zmp_x_[0]*100 << " zmp_y: " << current_state_.zmp_y_[0]*100 << std::endl;
                std::cout << "true_zmp_x: " << true_ZMP.x()*100 << " true_zmp_y: " << true_ZMP.y()*100 << std::endl;
                // ===================================================================
                zmp_y_.data = current_state_.zmp_y_[0];
                pub_zmp_y_.publish(zmp_y_);
                true_zmp_.data = true_ZMP.y();
                pub_true_zmp_.publish(true_zmp_);
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
