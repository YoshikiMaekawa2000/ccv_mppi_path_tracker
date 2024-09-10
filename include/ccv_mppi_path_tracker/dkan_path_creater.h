#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

class DkanPathCreater
{
public:
    DkanPathCreater();
    ~DkanPathCreater();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    geometry_msgs::PoseStamped pose_;
    std::vector<geometry_msgs::PoseStamped> poses_;

    geometry_msgs::PoseStamped poses_0;
    geometry_msgs::PoseStamped poses_1;
    geometry_msgs::PoseStamped poses_2;
    geometry_msgs::PoseStamped poses_3;

    double hz_;
    double resolution_;

    void init_poses();
    void add_pose_to_path(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
};


DkanPathCreater::~DkanPathCreater()
{
}