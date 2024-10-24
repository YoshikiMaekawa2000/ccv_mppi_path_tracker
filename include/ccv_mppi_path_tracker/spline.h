#include <math.h>
#include <vector>

using namespace std;

class CppCubicSpline
{
public:
    CppCubicSpline(const vector<double> &x, const vector<double> &y) { InitParameter(x, y); }

    double Calc(double t)
    {
        int j = BinarySearch(t);
        double dx = t - x_[j];
        double result = a_[j] + (b_[j] + (c_[j] + d_[j] * dx) * dx) * dx;
        return result;
    }

private:
    vector<double> x_;
    vector<double> a_;
    vector<double> b_;
    vector<double> c_;
    vector<double> d_;
    vector<double> w_;

    void InitParameter(const vector<double> &x, const vector<double> &y)
    {
        x_ = x;
        int ndata = y.size() - 1;

        for (int i = 0; i <= ndata; i++)
        {
            a_.push_back(y[i]);
        }

        for (int i = 0; i < ndata; i++)
        {
            if (i == 0)
            {
                c_.push_back(0.0);
            }
            else if (i == ndata)
            {
                c_.push_back(0.0);
            }
            else
            {
                double dx1 = x_[i] - x_[i - 1];
                double dx2 = x_[i + 1] - x_[i];
                c_.push_back(3.0 * (a_[i - 1] * dx2 - a_[i] * (dx1 + dx2) + a_[i + 1] * dx1) / (dx1 * dx2));
            }
        }

        for (int i = 0; i < ndata; i++)
        {
            if (i == 0)
            {
                w_.push_back(0.0);
            }
            else
            {
                double tmp = 4.0 - w_[i - 1];
                c_[i] = (c_[i] - c_[i - 1]) / tmp;
                w_.push_back(1.0 / tmp);
            }
        }

        for (int i = (ndata - 1); i > 0; i--)
        {
            c_[i] = c_[i] - c_[i + 1] * w_[i];
        }

        for (int i = 0; i <= ndata; i++)
        {
            if (i == ndata)
            {
                d_.push_back(0.0);
                b_.push_back(0.0);
            }
            else
            {
                double dx = x_[i + 1] - x_[i];
                d_.push_back((c_[i + 1] - c_[i]) / (3.0 * dx));
                b_.push_back((a_[i + 1] - a_[i]) / dx - dx * (c_[i] + d_[i] * dx));
            }
        }
    }

    int BinarySearch(double t)
    {
        int low = 0;
        int high = x_.size() - 1;
        while (high - low > 1)
        {
            int mid = (low + high) / 2;
            if (x_[mid] > t)
            {
                high = mid;
            }
            else
            {
                low = mid;
            }
        }
        return low;
    }
};
