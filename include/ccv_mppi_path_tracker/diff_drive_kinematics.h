class DiffDriveKinematics
{
public:
    DiffDriveKinematics(double x, double y, double theta, double vel);

    void updatePose(double v, double w);
    void getPose(double& x, double& y, double& theta);
private:

    double x_;
    double y_;
    double theta_;
    double vel_;
};

DiffDriveKinematics::DiffDriveKinematics(double x, double y, double theta, double vel)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
    vel_ = vel;
}

void DiffDriveKinematics::updatePose(double v, double w)
{
    x_ += v * cos(theta_);
    y_ += v * sin(theta_);
    theta_ += w;
}
