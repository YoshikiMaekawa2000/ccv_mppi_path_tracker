#include <random>

class DiffDrive{
public:
    DiffDrive();
    void init(int horizon);
    void sampling(double mu, double sigma);
    std::vector<double> x_, y_, yaw_;
    std::vector<double> v_, w_;
private:
    double Tread_;
    double wheel_radius_;
    int horizon_;
};

DiffDrive::DiffDrive(): Tread_(0.501), wheel_radius_(0.1435)
{
}
void DiffDrive::init(int horizon)
{
    horizon_ = horizon;
    x_.resize(horizon_);
    y_.resize(horizon_);
    yaw_.resize(horizon_);
    v_.resize(horizon_);
    w_.resize(horizon_);
    for(int i=0; i<horizon_; i++)
    {
        x_[i] = 0.0;
        y_[i] = 0.0;
        yaw_[i] = 0.0;
        v_[i] = 0.0;
        w_[i] = 0.0;
    }
}

void DiffDrive::sampling(double mu, double sigma)
{
    // Add Gaussian noise to control inputs
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::normal_distribution<> norm(mu, sigma);
}
