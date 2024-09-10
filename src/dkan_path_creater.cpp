#include "ccv_mppi_path_tracker/dkan_path_creater.h"

DkanPathCreater::DkanPathCreater(): nh_("~")
{
    path_pub_ = nh_.advertise<nav_msgs::Path>("/dkan_path_creater/path", 1);
    nh_.param("hz", hz_, 10.0);
    nh_.param("resolution", resolution_, 0.1);
    init_poses();
}

void DkanPathCreater::init_poses()
{
    //中央
    poses_0.header.frame_id = "odom";
    poses_0.pose.position.x = 0.0;
    poses_0.pose.position.y = 0.0;
    //上
    poses_1.header.frame_id = "odom";
    poses_1.pose.position.x = 17.7;
    poses_1.pose.position.y = 0.0;
    //左上。一番せまいところ
    poses_2.header.frame_id = "odom";
    poses_2.pose.position.x = 17.7;
    // poses_2.pose.position.y = 7.4;
    poses_2.pose.position.y = 8.0;
    //左
    poses_3.header.frame_id = "odom";
    poses_3.pose.position.x = 0.0;
    poses_3.pose.position.y = 8.0;
    
    poses_.push_back(poses_0);
    poses_.push_back(poses_1);
    poses_.push_back(poses_2);
    poses_.push_back(poses_3);
}

void DkanPathCreater::add_pose_to_path(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    for(double s = 0.0; s < sqrt(dx * dx + dy * dy); s += resolution_)
    {
        pose_.header.stamp = ros::Time::now();
        pose_.header.frame_id = "odom";
        pose_.pose.position.x = pose1.pose.position.x + s * dx / sqrt(dx * dx + dy * dy);
        pose_.pose.position.y = pose1.pose.position.y + s * dy / sqrt(dx * dx + dy * dy);
        pose_.pose.position.z = 0.0;
        pose_.pose.orientation.w = 1.0;
        path_.poses.push_back(pose_);
    }
    
}
void DkanPathCreater::run()
{
    ros::Rate loop_rate(hz_);
    double t = 0.0;
    while(ros::ok())
    {
        path_.header.stamp = ros::Time::now();
        path_.header.frame_id = "odom";
        path_.poses.clear();
        add_pose_to_path(poses_[0], poses_[1]);
        add_pose_to_path(poses_[1], poses_[2]);
        add_pose_to_path(poses_[2], poses_[3]);
        // add_pose_to_path(poses_[3], poses_[0]);
        
        path_pub_.publish(path_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dkan_path_creater");
    DkanPathCreater dkan_path_creater;
    dkan_path_creater.run();
    return 0;
}