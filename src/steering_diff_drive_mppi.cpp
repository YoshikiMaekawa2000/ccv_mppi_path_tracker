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
    nh_.param("w_max", w_max_, 1.0);
    nh_.param("steer_max", steer_max_, {30.0*M_PI/180.0});
    nh_.param("v_min", v_min_, -1.2);
    nh_.param("w_min", w_min_, -1.0);
    nh_.param("steer_min", steer_min_, {-30.0*M_PI/180.0});
    nh_.param("pitch_offset", pitch_offset_, {3.0*M_PI/180.0});
    nh_.param("v_ref", v_ref_, 1.2);
    nh_.param("resolution", resolution_, 0.1);
    nh_.param("exploration_noise", exploration_noise_, 0.1);
    nh_.param("world_frame", WORLD_FRAME, std::string("odom"));
    nh_.param("robot_frame", ROBOT_FRAME, std::string("base_link"));

    // Initialize sample[i]
    sample.resize(num_samples_);
    for (int i = 0; i < num_samples_; i++) sample[i].init(horizon_);
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
    if(!path_received_) path_received_ = true;
}
void SteeringDiffDriveMPPI::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    current_steer_l_ = msg->position[5];   //left
    current_steer_r_ = msg->position[7];   //right

    if(!joint_state_received_) joint_state_received_ = true;
    std::string state = check_State(current_steer_r_, current_steer_l_);
    if(state == "no_need")  ROS_ERROR("===== Current Steer Angle is Invalid =====");
}
void SteeringDiffDriveMPPI::clamp(double &val, double min, double max)
{
    if(val < min) val = min;
    else if(val > max) val = max;
    
}
std::string SteeringDiffDriveMPPI::check_State(double steer_r, double steer_l)
{
    std::string state;
    double eps = 0.1*M_PI/180.0;
    if(steer_r < 0.0 && steer_l > 0.0 ||steer_r > 0.0 && steer_l < 0.0)  state = "no_need";        //ハの字，逆ハの字の場合は使わない
    else if(fabs(steer_r - steer_l) < eps){
        if(fabs(steer_r) < eps && fabs(steer_l) < eps)  state = "no_steer";                             //通常の差動二輪と同じ
        else  state = "parallel";                                                                       //左右のステアが平行の場合
    }                            
    else  state = "steer";                                                                              //ステアを切っている場合
    return state;
}

void SteeringDiffDriveMPPI::sampling()
{
    std::random_device rnd;
    std::mt19937 mt(rnd());

    for(int t=0; t < horizon_-1; t++)
    {
        std::normal_distribution<> norm_v(optimal_solution.v_[t], control_noise_);
        std::normal_distribution<> norm_w(optimal_solution.w_[t], control_noise_);
        std::normal_distribution<> norm_steer(optimal_solution.steer_[t], control_noise_);

        for(int i=0; i < num_samples_; i++)
        {
            sample[i].v_[t] = norm_v(mt);
            sample[i].w_[t] = norm_w(mt);
            sample[i].steer_[t] = norm_steer(mt);
            clamp(sample[i].v_[t], v_min_, v_max_);
            clamp(sample[i].w_[t], w_min_, w_max_);
            clamp(sample[i].steer_[t], steer_min_, steer_max_);
        }
    }
}

void SteeringDiffDriveMPPI::predict_NextState(RobotStates &sample, int t)
{
    sample.x_[t+1] = sample.x_[t] + sample.v_[t] * cos(sample.yaw_[t] + sample.steer_[t]) * dt_;
    sample.y_[t+1] = sample.y_[t] + sample.v_[t] * sin(sample.yaw_[t] + sample.steer_[t]) * dt_;
    sample.yaw_[t+1] = sample.yaw_[t] + sample.w_[t] * dt_;
}

void SteeringDiffDriveMPPI::predict_States()
{
    for(int i=0; i < num_samples_; i++)
    {
        sample[i].x_[0] = current_pose_.pose.position.x;
        sample[i].y_[0] = current_pose_.pose.position.y;
        sample[i].yaw_[0] = tf::getYaw(current_pose_.pose.orientation);

        for(int t=0; t<horizon_-1; t++){
            predict_NextState(sample[i], t);
        }
    }
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
    current_index_ = get_CurrentIndex();

    double step = v_ref_ * dt_ / resolution_;
    // int step = 1;
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
        double v_cost = v_ref_ - sample.v_[t];
        cost += dx*dx + dy*dy + v_cost*v_cost;
        // cost += dx*dx + dy*dy;
    }
    return cost;
}

void SteeringDiffDriveMPPI::calc_Weights()
{
    calc_RefPath();
    double sum = 0.0;
    for(int i=0; i<num_samples_; i++)
    {
        double cost = calc_Cost(sample[i]);
        weights_[i] = exp(-cost / lambda_);
        sum += weights_[i];
    }
    for(int i=0; i<num_samples_; i++) weights_[i] /= sum;
}

void SteeringDiffDriveMPPI::determine_OptimalSolution()
{
    // Determine optimal solution
    for(int t=0; t<horizon_; t++)
    {
        optimal_solution.v_[t] = 0.0;
        optimal_solution.w_[t] = 0.0;
        optimal_solution.steer_[t] = 0.0;
        for(int i=0; i<num_samples_; i++)
        {
            optimal_solution.v_[t] += weights_[i] * sample[i].v_[t];
            optimal_solution.w_[t] += weights_[i] * sample[i].w_[t];
            optimal_solution.steer_[t] += weights_[i] * sample[i].steer_[t];
        }
    }
    // Publish optimal path
    std::cout << "====================" << std::endl;
    std::cout << "v: " << optimal_solution.v_[0] << " w: " << optimal_solution.w_[0] << " steer: " << optimal_solution.steer_[0]*180/M_PI << std::endl;

    publish_OptimalPath();
}

void SteeringDiffDriveMPPI::publish_CmdVel()
{
    cmd_vel_.linear.x = optimal_solution.v_[0];
    cmd_vel_.angular.z = optimal_solution.w_[0];
    pub_cmd_vel_.publish(cmd_vel_);
}

void SteeringDiffDriveMPPI::publish_CmdPos()
{
    double R = fabs(optimal_solution.v_[0] / optimal_solution.w_[0]);
    double steer_in = std::atan2(R*sin(optimal_solution.steer_[0]), R*cos(optimal_solution.steer_[0]) - tread_/2.0);
    double steer_out = std::atan2(R*sin(optimal_solution.steer_[0]), R*cos(optimal_solution.steer_[0]) + tread_/2.0);

    std::cout << "R: " << R << std::endl;
    std::cout << "steer_in: " << steer_in*180/M_PI << " steer_out: " << steer_out*180/M_PI << std::endl;

    if(optimal_solution.w_[0] > 0.0){
        cmd_pos_.steer_l = steer_in;
        cmd_pos_.steer_r = steer_out;
    }
    else{
        cmd_pos_.steer_l = steer_out;
        cmd_pos_.steer_r = steer_in;
    }
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
    optimal_path_.poses.resize(horizon_-1);
    optimal_solution.x_[0] = current_pose_.pose.position.x;
    optimal_solution.y_[0] = current_pose_.pose.position.y;
    optimal_solution.yaw_[0] = tf::getYaw(current_pose_.pose.orientation);
    for(int i=0; i<horizon_-1; i++)
    {
        predict_NextState(optimal_solution, i);
        optimal_path_.poses[i].pose.position.x = optimal_solution.x_[i];
        optimal_path_.poses[i].pose.position.y = optimal_solution.y_[i];
        optimal_path_.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(optimal_solution.yaw_[i]);
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
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }       
}

// void SteeringDiffDriveMPPI::save_Data()
// {
//     // Save data
//     if(first_save_) {
//         ofs.open("/home/amsl/catkin_ws/src/ccv_mppi_path_tracker/data/data.csv", std::ios::out);
//         first_save_ = false;
//     }
//     ofs << optimal_solution.steer_l_[0] *180/M_PI<< ",";
//     ofs << optimal_solution.steer_r_[0] *180/M_PI<< "," << std::endl;
// }

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
    ros::init(argc, argv, "steering_diff_drive_mppi");
    SteeringDiffDriveMPPI steering_diff_drive_mppi;
    steering_diff_drive_mppi.run();
    return 0;
}