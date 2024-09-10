#include <iostream>
#include <random>
class SteeringDiffDrive{
    public:
        SteeringDiffDrive();
        void init(int horizon);
        std::vector<double> x_, y_, yaw_, direction_;
        std::vector<double> v_, w_, steer_left_, steer_right_;
    private:
        double Tread_;
        double wheel_radius_;
        int horizon_;
};

SteeringDiffDrive::SteeringDiffDrive(): Tread_(0.501), wheel_radius_(0.1435)
{
}

void SteeringDiffDrive::init(int horizon)
{
    horizon_ = horizon;
    x_.resize(horizon_);
    y_.resize(horizon_);
    yaw_.resize(horizon_);
    direction_.resize(horizon_);
    v_.resize(horizon_);
    w_.resize(horizon_);
    steer_left_.resize(horizon_);
    steer_right_.resize(horizon_);
    for(int i=0; i<horizon_; i++)
    {
        x_[i] = 0.0;
        y_[i] = 0.0;
        yaw_[i] = 0.0;
        direction_[i] = 0.0;
        v_[i] = 0.0;
        w_[i] = 0.0;
        steer_left_[i] = 0.0;
        steer_right_[i] = 0.0;
    }
}

