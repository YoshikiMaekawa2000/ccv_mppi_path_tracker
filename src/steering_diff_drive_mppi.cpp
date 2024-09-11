#include "ccv_mppi_path_tracker/steering_diff_drive_mppi.h"

SteeringDiffDriveMPPI::SteeringDiffDriveMPPI()
    : nh_("~"), path_received_(false), joint_state_received_(false), first_roop_(true), tread_(0.501), wheel_radius_(0.1435)
{
    // publishers
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);
    // subscribers
    sub_path_ = nh_.subscribe("/reference_path", 1, &SteeringDiffDriveMPPI::pathCallback, this);
    sub_joint_state_ = nh_.subscribe("/sq2_ccv/joint_states", 1, &SteeringDiffDriveMPPI::jointStateCallback, this);
    // for debug
    pub_ref_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/ref_path", 1);
    pub_candidate_path_ = nh_.advertise<visualization_msgs::MarkerArray>("/ccv_mppi_path_tracker/candidate_path", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/optimal_path", 1);

    nh_.param("dt", dt_, 0.1);
    nh_.param("horizon", horizon_, 15);
    nh_.param("num_samples", num_samples_, 1000.0);
    nh_.param("control_noise", control_noise_, 0.5);
    nh_.param("lambda", lambda_, 1.0);
    nh_.param("v_max", v_max_, 1.2);
    // nh_.param("w_max", w_max_, M_PI/2.0);
    nh_.param("steer_max", steer_max_, {30.0*M_PI/180.0});
    // nh_.param("steer_max", steer_max_, 30.0);
    nh_.param("v_min", v_min_, -1.2);
    // nh_.param("w_min", w_min_, -M_PI/2.0);
    nh_.param("steer_min", steer_min_, {-30.0*M_PI/180.0});
    // nh_.param("steer_min", steer_min_, -30.0);
    nh_.param("pitch_offset", pitch_offset_, {3.0*M_PI/180.0});
    nh_.param("v_ref", v_ref_, 1.2);
    nh_.param("resolution", resolution_, 0.1);
    nh_.param("exploration_noise", exploration_noise_, 0.1);
    nh_.param("world_frame", WORLD_FRAME, std::string("odom"));
    nh_.param("robot_frame", ROBOT_FRAME, std::string("base_link"));

    // Initialize sample
    sample.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++)
    {
        sample[i].init(horizon_);
    }
    // Initialize optimal_solution
    optimal_solution.init(horizon_);
    // Initialize reference path
    x_ref_.resize(horizon_);
    y_ref_.resize(horizon_);
    yaw_ref_.resize(horizon_);
    // Initialize weights
    weights_.resize(num_samples_);
}

void SteeringDiffDriveMPPI::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
    if(!path_received_)
    {
        path_received_ = true;
    }
}

void SteeringDiffDriveMPPI::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    current_steer_in_ = msg->position[5];   //left
    current_steer_out_ = msg->position[7];   //right

    if(fabs(current_steer_in_) < fabs(current_steer_out_))
    {
        double tmp = current_steer_in_;
        current_steer_in_ = current_steer_out_;
        current_steer_out_ = tmp;
    }
    if(!joint_state_received_)
    {
        joint_state_received_ = true;
    }
}

void SteeringDiffDriveMPPI::clamp(double &val, double min, double max)
{
    if(val < min)
    {
        val = min;
    }
    else if(val > max)
    {
        val = max;
    }
}

void SteeringDiffDriveMPPI::check_Samples()
{
    int no_need_samples = 0;
    int no_steer_samples = 0;
    int steer_samples = 0;
    int parallel_samples = 0;
    double eps = 0.1*M_PI/180.0;
    double steer_r_min = steer_max_;
    double steer_l_min = steer_max_;
    double steer_r_max = steer_min_;
    double steer_l_max = steer_min_;
    double mu_r = 0.0;
    double mu_l = 0.0;
    for(int i=0; i < num_samples_; i++)
    {
        for(int t=0; t < horizon_-1; t++)
        {
            mu_r += sample[i].steer_r_[t];
            mu_l += sample[i].steer_l_[t];
            if(sample[i].steer_r_[t] < steer_r_min) steer_r_min = sample[i].steer_r_[t];
            if(sample[i].steer_l_[t] < steer_l_min) steer_l_min = sample[i].steer_l_[t];
            if(sample[i].steer_r_[t] > steer_r_max) steer_r_max = sample[i].steer_r_[t];
            if(sample[i].steer_l_[t] > steer_l_max) steer_l_max = sample[i].steer_l_[t];

            if(sample[i].steer_r_[t] < 0.0 && sample[i].steer_l_[t] > 0.0 || sample[i].steer_r_[t] > 0.0 && sample[i].steer_l_[t] < 0.0)  no_need_samples++;  //ハの字 or 逆ハの字の場合
            else if(fabs(sample[i].steer_r_[t] - sample[i].steer_l_[t]) < eps) parallel_samples++;                                          //左右のステアが平行の場合
            else if(fabs(sample[i].steer_r_[t]) < eps && fabs(sample[i].steer_l_[t]) < eps)  no_steer_samples++;                            //通常の差動二輪と同じ
            else  steer_samples++;                                                                                                         //ステアがある場合
        }
    }
    
    std::cout << "=====================" << std::endl;
    std::cout << "no_need_samples: " << no_need_samples * 100/(num_samples_*(horizon_-1)) << "  no_steer_samples: " << no_steer_samples*100/(num_samples_*(horizon_-1))
    << "  steer_samples: " << steer_samples*100/(num_samples_*(horizon_-1)) << "  parallel_samples: " << parallel_samples*100/(num_samples_*(horizon_-1)) << std::endl;
    std::cout << "mu_r: " << (mu_r*180/M_PI)/(num_samples_*(horizon_-1)) << "  mu_l: " << (mu_l*180/M_PI)/(num_samples_*(horizon_-1)) << std::endl;
    std::cout << "steer_r_min: " << steer_r_min*180/M_PI << "  steer_l_min: " << steer_l_min*180/M_PI << std::endl;
    std::cout << "steer_r_max: " << steer_r_max*180/M_PI << "  steer_l_max: " << steer_l_max*180/M_PI << std::endl;
}

void SteeringDiffDriveMPPI::sampling()
{
    // Add Gaussian noise to the control inputs
    std::random_device rnd;
    std::mt19937 mt(rnd());

    for(int t=0; t < horizon_-1; t++)
    {
        std::normal_distribution<> norm_vr(optimal_solution.vr_[t], control_noise_);
        std::normal_distribution<> norm_vl(optimal_solution.vl_[t], control_noise_);
        std::normal_distribution<> norm_steer_r(optimal_solution.steer_r_[t], 0.1 * control_noise_);
        std::normal_distribution<> norm_steer_l(optimal_solution.steer_l_[t], 0.1 * control_noise_);

        for(int i=0; i < num_samples_; i++)
        {
            sample[i].vr_[t] = norm_vr(mt);
            sample[i].vl_[t] = norm_vl(mt);
            sample[i].steer_r_[t] = norm_steer_r(mt);
            sample[i].steer_l_[t] = norm_steer_l(mt);
            // Clamp the control inputs
            clamp(sample[i].vr_[t], v_min_, v_max_);
            clamp(sample[i].vl_[t], v_min_, v_max_);
            //あとで w に関するclampを追加

            double w = (sample[i].vr_[t] - sample[i].vl_[t]) / tread_;
            if(w > 0.0)
            {
                clamp(sample[i].steer_r_[t], 0.0, steer_max_);
                clamp(sample[i].steer_l_[t], 0.0, steer_max_);
            }
            else
            {
                clamp(sample[i].steer_r_[t], steer_min_, 0.0);
                clamp(sample[i].steer_l_[t], steer_min_, 0.0);
            }
        }
    }
    //for debug
    check_Samples();
}

double SteeringDiffDriveMPPI::calc_Omega(double vr, double vl, double steer_r, double steer_l)
{
    
    double R_r = sin(fabs(steer_r)) * tread_ / sin(fabs(steer_r - steer_l));
    double R_l = sin(fabs(steer_l)) * tread_ / sin(fabs(steer_r - steer_l));
    return (vr - vl) / fabs(R_r - R_l);
    
}

void SteeringDiffDriveMPPI::predict_States()
{
    for(int i=0; i < num_samples_; i++)
    {
        sample[i].x_[0] = current_pose_.pose.position.x;
        sample[i].y_[0] = current_pose_.pose.position.y;
        sample[i].yaw_[0] = tf::getYaw(current_pose_.pose.orientation);

        for(int t=0; t < horizon_-1; t++)
        {
            double eps = 0.1*M_PI/180.0;
            double vx = (sample[i].vr_[t] * cos(sample[i].steer_r_[t]) + sample[i].vl_[t] * cos(sample[i].steer_l_[t])) / 2.0;
            double vy = (sample[i].vr_[t] * sin(sample[i].steer_r_[t]) + sample[i].vl_[t] * sin(sample[i].steer_l_[t])) / 2.0;
            sample[i].x_[t+1] = sample[i].x_[t] + vx * cos(sample[i].yaw_[t]) * dt_ - vy * sin(sample[i].yaw_[t]) * dt_;
            sample[i].y_[t+1] = sample[i].y_[t] + vx * sin(sample[i].yaw_[t]) * dt_ + vy * cos(sample[i].yaw_[t]) * dt_;

            if(fabs(sample[i].steer_r_[t]) < eps && fabs(sample[i].steer_l_[t]) < eps)    //通常の差動二輪と同じ
            {
                double omega = (sample[i].vr_[t] - sample[i].vl_[t]) / tread_;
                sample[i].yaw_[t+1] = sample[i].yaw_[t] + omega * dt_;
            }
            else if(fabs(sample[i].steer_r_[t] - sample[i].steer_l_[t]) < eps)           //左右のステアが平行の場合
            {
                sample[i].yaw_[t+1] = sample[i].yaw_[t];
            }
            else                                                                        //ステアを切った場合
            {
                double omega = calc_Omega(sample[i].vr_[t], sample[i].vl_[t], sample[i].steer_r_[t], sample[i].steer_l_[t]);
                sample[i].yaw_[t+1] = sample[i].yaw_[t] + omega * dt_;
            }
        }
    }
    // Publish candidate path
    publish_CandidatePath();
}

int SteeringDiffDriveMPPI::get_CurrentIndex()
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

void SteeringDiffDriveMPPI::publish_RefPath()
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

void SteeringDiffDriveMPPI::calc_RefPath()
{
    // Calculate reference path
    current_index_ = get_CurrentIndex();

    //あとで直す
    // double step = v_ref_ * dt_ / resolution_;
    int step = 1;
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

double SteeringDiffDriveMPPI::calc_Cost(RobotStates sample)
{
    // Calculate cost
    double cost = 0.0;
    for(int t=0; t < horizon_; t++)
    {
        double dx = sample.x_[t] - x_ref_[t];
        double dy = sample.y_[t] - y_ref_[t];
        double dyaw = sample.yaw_[t] - yaw_ref_[t];
        // double v_cost = v_ref_ - (sample.vr_[t] + sample.vl_[t]) / 2.0;
        cost += dx*dx + dy*dy + dyaw*dyaw;
        // cost += dyaw*dyaw + v_cost*v_cost;
    }
    return cost;
}

void SteeringDiffDriveMPPI::calc_Weights()
{
    //calcurate reference path
    calc_RefPath();

    // Calculate weights
    double sum = 0.0;
    for(int i=0; i<num_samples_; i++)
    {
        // Calculate cost
        double cost = calc_Cost(sample[i]);

        weights_[i] = exp(-cost / lambda_);
        sum += weights_[i];
    }
    // Normalize weights
    for(int i=0; i<num_samples_; i++)
    {
        weights_[i] /= sum;
    }
}

void SteeringDiffDriveMPPI::determine_OptimalSolution()
{
    // Determine optimal solution
    for(int t=0; t<horizon_; t++)
    {
        optimal_solution.vr_[t] = 0.0;
        optimal_solution.vl_[t] = 0.0;
        optimal_solution.steer_r_[t] = 0.0;
        optimal_solution.steer_l_[t] = 0.0;
        for(int i=0; i<num_samples_; i++)
        {
            optimal_solution.vr_[t] += weights_[i] * sample[i].vr_[t];
            optimal_solution.vl_[t] += weights_[i] * sample[i].vl_[t];
            optimal_solution.steer_r_[t] += weights_[i] * sample[i].steer_r_[t];
            optimal_solution.steer_l_[t] += weights_[i] * sample[i].steer_l_[t];
        }
    }
    // for debug
    if(optimal_solution.steer_r_[0] > 0.0 && optimal_solution.steer_l_[0] < 0.0 || optimal_solution.steer_r_[0] < 0.0 && optimal_solution.steer_l_[0] > 0.0)
    {
        std::cout << "steer_r: " << optimal_solution.steer_r_[0]*180/M_PI << "  steer_l: " << optimal_solution.steer_l_[0]*180/M_PI <<"  Steering Angle Error"<<std::endl;
    }
    else std::cout << "steer_r: " << optimal_solution.steer_r_[0]*180/M_PI<< "  steer_l: " << optimal_solution.steer_l_[0]*180/M_PI << std::endl;
    // publish_OptimalPath();
}

void SteeringDiffDriveMPPI::publish_CmdVel()
{
    cmd_vel_.linear.x = (optimal_solution.vr_[0] + optimal_solution.vl_[0]) / 2.0;
    std::cout << "v: " << cmd_vel_.linear.x << std::endl;
    cmd_vel_.angular.z = (optimal_solution.vr_[0] - optimal_solution.vl_[0]) / tread_;
    // cmd_vel_.linear.x = 0.0;
    // cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
}

void SteeringDiffDriveMPPI::publish_CmdPos()
{
    cmd_pos_.steer_l = optimal_solution.steer_l_[0];
    cmd_pos_.steer_r = optimal_solution.steer_r_[0];
    cmd_pos_.fore=pitch_offset_;
    cmd_pos_.rear=pitch_offset_;
    cmd_pos_.roll=0.0;
    pub_cmd_pos_.publish(cmd_pos_);
}

void SteeringDiffDriveMPPI::publish_CandidatePath()
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
void SteeringDiffDriveMPPI::publish_OptimalPath()
{
    optimal_path_.header.frame_id = WORLD_FRAME;
    optimal_path_.header.stamp = ros::Time::now();
    optimal_path_.poses.resize(horizon_);
    for(int i=0; i<horizon_; i++)
    {
        optimal_path_.poses[i].pose.position.x = sample[0].x_[i];
        optimal_path_.poses[i].pose.position.y = sample[0].y_[i];
        optimal_path_.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(sample[0].yaw_[i]);
    }
    pub_optimal_path_.publish(optimal_path_);
}

void SteeringDiffDriveMPPI::get_Transform()
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
        // std::cout << "x: " << current_pose_.pose.position.x << std::endl;
        // std::cout << "y: " << current_pose_.pose.position.y << std::endl;
        // std::cout << "yaw: " << tf::getYaw(current_pose_.pose.orientation) << std::endl;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }       
}

void SteeringDiffDriveMPPI::run()
{
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        if(path_received_)
        {
            if(first_roop_)
            {
                last_time_ = ros::Time::now().toSec();
                first_roop_ = false;
            }
            else{
                current_time_ = ros::Time::now().toSec();
                dt_ = current_time_ - last_time_;
                last_time_ = current_time_;
                // 0. Get transform
                get_Transform();
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
    ros::init(argc, argv, "steering_diff_drive_mppi");
    SteeringDiffDriveMPPI steering_diff_drive_mppi;
    steering_diff_drive_mppi.run();
    return 0;
}