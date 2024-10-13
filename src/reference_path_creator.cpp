#include "ccv_mppi_path_tracker/reference_path_creater.h"

ReferencePathCreater::ReferencePathCreater(): nh_("~")
{
    path_pub_ = nh_.advertise<nav_msgs::Path>("/reference_path_creater/path", 1);
    nh_.param("A1", A1_, 0.0);
    nh_.param("A2", A2_, 0.0);
    nh_.param("A3", A3_, 0.0);
    nh_.param("omega1", omega1_, 0.0);
    nh_.param("omega2", omega2_, 0.0);
    nh_.param("omega3", omega3_, 0.0);
    nh_.param("delta1", delta1_, 1.57);
    nh_.param("delta2", delta2_, 1.57);
    nh_.param("delta3", delta3_, 1.57);
    nh_.param("hz", hz_, 10.0);
    nh_.param("resolution", resolution_, 0.1);
    nh_.param("course_length", course_length_, 10.0);
    nh_.param("init_x", init_x_, 0.0);
    nh_.param("init_y", init_y_, 0.0);
    nh_.param("init_yaw", init_yaw_, 0.0);
    nh_.param("world_frame", world_frame_, std::string("odom"));
}
void ReferencePathCreater::run()
{
    ros::Rate loop_rate(hz_);
    double t = 0.0;
    std::cout << "2*M_PI*omega1_: " << 2*M_PI*omega1_ << std::endl;
    while(ros::ok())
    {
        path_.header.stamp = ros::Time::now();
        path_.header.frame_id = world_frame_;
        path_.poses.clear();
        for(double s = 0.0; s < course_length_; s += resolution_)
        {
            pose_.header.stamp = ros::Time::now();
            pose_.header.frame_id = world_frame_;
            pose_.pose.position.x = init_x_ + s;
            // pose_.pose.position.y = A1_ * sin(omega1_ * s + delta1_) + A2_ * sin(omega2_ * s + delta2_) + A3_ * sin(omega3_ * s + delta3_) + init_y_;
            // pose_.pose.position.y = A1_ * sin(2*M_PI*omega1_ * s + delta1_) + A2_ * sin(2*M_PI*omega2_ * s + delta2_) + A3_ * sin(2*M_PI*omega3_ * s + delta3_) + init_y_;
            pose_.pose.position.y = A1_ * cos(2*M_PI*omega1_ * s + delta1_) + A2_ * cos(2*M_PI*omega2_ * s + delta2_) + A3_ * cos(2*M_PI*omega3_ * s + delta3_) + init_y_;
            pose_.pose.position.y -=A1_ + A2_ + A3_;
            pose_.pose.position.z = 0.0;
            pose_.pose.orientation.w = 1.0;
            path_.poses.push_back(pose_);
        }
        path_pub_.publish(path_);
        ros::spinOnce();
        loop_rate.sleep();
        t += 1.0 / hz_;
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "reference_path_creater");
    ReferencePathCreater reference_path_creater;
    reference_path_creater.run();
    return 0;
}