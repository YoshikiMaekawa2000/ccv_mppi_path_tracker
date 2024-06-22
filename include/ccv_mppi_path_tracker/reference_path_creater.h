#include <ros/ros.h>
#include <nav_msgs/Path.h>

class ReferencePathCreater
{
public:
    ReferencePathCreater();
    ~ReferencePathCreater();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    geometry_msgs::PoseStamped pose_;

    double A1_, A2_, A3_;
    double omega1_, omega2_, omega3_;
    double delta1_, delta2_, delta3_;
    double hz_;
    double resolution_;
    double course_length_;
    double init_x_, init_y_, init_yaw_;
    std::string world_frame_;
};

ReferencePathCreater::~ReferencePathCreater()
{
}