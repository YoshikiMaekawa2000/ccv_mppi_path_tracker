
#include "ccv_mppi_path_tracker/ccv_mppi_path_tracker.h"

CCVMPPIPathTracker::CCVMPPIPathTracker(): nh_("~"), path_received_(false), joint_state_received_(false), transformed_(false)
{
    path_pub_ = nh_.advertise<nav_msgs::Path>("/ccv_mppi_path_tracker/path", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ccv_mppi_path_tracker/cmd_vel", 1);
    path_sub_ = nh_.subscribe("/reference_path_creater/path", 1, &CCVMPPIPathTracker::pathCallback, this);
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &CCVMPPIPathTracker::jointStateCallback, this);

    nh_.param("dt", dt_, 0.1);
    nh_.param("horizon", horizon_, 10.0);
    nh_.param("num_samples", num_samples_, 100.0);
    nh_.param("control_noise", control_noise_, 0.1);
    nh_.param("exploration_noise", exploration_noise_, 0.1);
    nh_.param("world_frame", WORLD_FRAME, std::string("odom"));
    nh_.param("robot_frame", ROBOT_FRAME, std::string("base_link"));
}

void CCVMPPIPathTracker::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_ = *msg;
    if(!path_received_)
    {
        path_received_ = true;
    }

}
void CCVMPPIPathTracker::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state_ = *msg;
    if(!joint_state_received_)
    {
        joint_state_received_ = true;
    }
}

void CCVMPPIPathTracker::getTransform()
{

    try{
        listener_.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), transform_);
        tf::transformStampedTFToMsg(transform_, transform_msg_);
        current_pose_.header = transform_msg_.header;
        current_pose_.pose.position.x = transform_msg_.transform.translation.x;
        current_pose_.pose.position.y = transform_msg_.transform.translation.y;
        current_pose_.pose.orientation = transform_msg_.transform.rotation;
        transformed_ = true;
        std::cout << "===Transformed===" << std::endl;
    }catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
    }
}
void CCVMPPIPathTracker::sampling()
{
}

void CCVMPPIPathTracker::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // std::cout << "===Running===" << std::endl;
        getTransform();
        if(path_received_ && joint_state_received_)
        {
            // Implement MPPI here
            // 0. Get transform
            // getTransform();
            // 1. Sample control inputs
            // sampling();
            // 2. Update pose
            // 3. Calculate weights
            // 4. Update path
            // 5. Publish path and cmd_vel
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ccv_mppi_path_tracker");
    CCVMPPIPathTracker ccv_mppi_path_tracker;
    ccv_mppi_path_tracker.run();
    return 0;
}